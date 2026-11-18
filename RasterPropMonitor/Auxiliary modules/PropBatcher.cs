using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Rendering;

namespace JSI
{
    public interface IPropBatchModuleHandler
    {
        /// <summary>
        /// 
        /// </summary>
        /// <param name="transform"></param>
        /// <returns>true if the module should be deleted from the prop</returns>
        bool NotifyTransformBatched(Transform transform);
    }

    public class PropBatcher : InternalModule
    {
        static Dictionary<string, BatchFilter> x_propNameToFilter = new Dictionary<string, BatchFilter>();
        static Dictionary<string, BatchFilter> x_modelNameToFilter = new Dictionary<string, BatchFilter>();

        // A batch filter selects transforms from a prop to associate with a given batch ID
        class BatchFilter
        {
            public BatchFilter(ConfigNode node, int batchID)
            {
                this.batchID = batchID;
                transformName = node.GetValue(nameof(transformName));
                node.TryGetValue(nameof(keepProp), ref keepProp);

                string propName = node.GetValue(nameof(propName));
                if (propName != null && !TryAdd(x_propNameToFilter, propName, this))
                {
                    JUtil.LogErrorMessage(null, "PROP_BATCH: Tried to add prop {0} to multiple batches", propName);
                }

                // models added with a MODEL node get their full path plus "(Clone)" as a top-level child of the "model" child of the prop
                string modelPath = node.GetValue(nameof(modelPath));
                if (modelPath != null && !TryAdd(x_modelNameToFilter, modelPath + "(Clone)", this))
                {
                    JUtil.LogErrorMessage(null, "PROP_BATCH: Tried to add modelPath {0} to multiple batches", modelPath);
                }

                // models that are implicitly added to props by virtue of being in the same directory are just directly added as children of the "model" node
                string modelName = node.GetValue(nameof(modelName));
                if (modelName != null &&  !TryAdd(x_modelNameToFilter, modelName, this))
                {
                    JUtil.LogErrorMessage(null, "PROP_BATCH: Tried to add modelName {0} to multiple batches", modelName);
                }
            }

            public readonly int batchID;
            public readonly string transformName;
            public readonly bool keepProp = true;
        }

        static bool TryAdd(Dictionary<string, BatchFilter> dict, string key, BatchFilter value)
        {
            if (dict.ContainsKey(key))
            {
                return false;
            }
            dict.Add(key, value);
            return true;
        }

        public static void ModuleManagerPostLoad()
        {
            x_propNameToFilter.Clear();
            x_modelNameToFilter.Clear();

            var batchNodes = GameDatabase.Instance.GetConfigNodes("PROP_BATCH");
            int nextBatchID = 0;
            foreach (var batchNode in batchNodes)
            {
                foreach (var filterNode in batchNode.GetNodes("FILTER"))
                {
                    new BatchFilter(filterNode, nextBatchID);
                }

                ++nextBatchID;
            }
        }

        static IEnumerable<BatchFilter> GetBatchFilterForProp(InternalProp prop)
        {
            if (!prop.hasModel) yield break;

            if (x_propNameToFilter.TryGetValue(prop.propName, out var batchFilter))
            {
                yield return batchFilter;
            }

            var modelRoot = prop.transform.Find("model");
            for (int childIndex = 0; childIndex < modelRoot.childCount; ++childIndex)
            {
                var model = modelRoot.GetChild(childIndex);
                if (x_modelNameToFilter.TryGetValue(model.name, out batchFilter))
                {
                    yield return batchFilter;
                }
            }
        }

        public override void OnLoad(ConfigNode node)
        {
            if (HighLogic.LoadedScene != GameScenes.LOADING) return;

            var newProps = new List<InternalProp>(internalModel.props.Count);
            var batchLists = new Dictionary<int, List<GameObject>>();
            int removedModuleCount = 0;
            int batchedTransformCount = 0;

            // collect batched transforms into the batchLists, and remove redundant props
            foreach (var oldProp in internalModel.props)
            {
                bool keepProp = true;
                foreach (var batchFilter in GetBatchFilterForProp(oldProp))
                {
                    var childTransform = JUtil.FindPropTransform(oldProp, batchFilter.transformName);
                    if (childTransform != null)
                    {
                        // get or create the batch list
                        if (batchLists.TryGetValue(batchFilter.batchID, out var batchList))
                        {
                            keepProp = batchFilter.keepProp;

                            if (keepProp)
                            {
                                // notify any modules that might need to know about this and remove them if necessary
                                for (int i = oldProp.internalModules.Count - 1; i >= 0; --i)
                                {
                                    if (oldProp.internalModules[i] is IPropBatchModuleHandler batchHandler)
                                    {
                                        if (batchHandler.NotifyTransformBatched(childTransform))
                                        {
                                            Component.Destroy(oldProp.internalModules[i]);
                                            oldProp.internalModules.RemoveAt(i);
                                            ++removedModuleCount;
                                        }
                                    }
                                }
                            }
                            else
                            {
                                // if we're not going to keep this prop, attach the transform to the batch root
                                childTransform.SetParent(batchList[0].transform.parent, true);
                            }
                        }
                        else
                        {
                            batchList = new List<GameObject>();
                            batchLists[batchFilter.batchID] = batchList;

                            // set up a batch root so we can attach all the other prop transforms to this one
                            if (!batchFilter.keepProp)
                            {
                                childTransform.SetParent(null, true); // detach early so the transform change below doesn't move it
                                oldProp.transform.localPosition = Vector3.zero;
                                oldProp.transform.localRotation = Quaternion.identity;
                                oldProp.transform.localScale = Vector3.one;
                                var batchRoot = new GameObject(oldProp.propName + " batchRoot").transform;
                                batchRoot.gameObject.layer = 20;
                                batchRoot.SetParent(oldProp.transform, false);
                                childTransform.SetParent(batchRoot.transform, true);
                            }
                        }

                        batchList.Add(childTransform.gameObject);
                    }
                    else
                    {
                        JUtil.LogErrorMessage(null, "PROP_BATCH: Could not find transform named {0} in prop {1}", batchFilter.transformName, oldProp.propName);
                    }
                }

                // if the prop doesn't have any modules left (or never had any to start),
                // just attach it to the internal model and remove the prop so we can avoid calling update etc.
                if (keepProp && oldProp.internalModules.Count == 0)
                {
                    var internalModelTransform = internalModel.transform.Find("model");
                    for (int i = oldProp.transform.childCount-1; i >= 0; i--)
                    {
                        oldProp.transform.GetChild(i).SetParent(internalModelTransform, true);
                    }

                    JUtil.LogMessage(null, "PROP_BATCH: removing prop {0} because it has no modules left", oldProp.propName);
                    keepProp = false;
                }

                if (keepProp)
                {
                    oldProp.propID = newProps.Count;
                    newProps.Add(oldProp);
                }
                else
                {
                    GameObject.Destroy(oldProp.gameObject);
                }
            }

            if (batchLists.Count > 0)
            {
                // merge batched meshes
                foreach (var batchList in batchLists.Values)
                {
                    CombineInstance[] instances = new CombineInstance[batchList.Count];
                    var worldToLocal = batchList[0].transform.worldToLocalMatrix;
                    
                    for (int i = 0; i < batchList.Count; i++)
                    {
                        var instance = new CombineInstance();
                        instance.mesh = batchList[i].GetComponent<MeshFilter>().mesh;
                        instance.transform = worldToLocal * batchList[i].transform.localToWorldMatrix;
                        
                        if (i > 0)
                        {
                            Component.Destroy(batchList[i].GetComponent<MeshRenderer>());
                            Component.Destroy(batchList[i].GetComponent<MeshFilter>());
                        }

                        instances[i] = instance;
                    }

                    batchedTransformCount += batchList.Count;
                    
                    var mesh = new Mesh();
                    CombineMeshes(mesh, instances);
                    batchList[0].GetComponent<MeshFilter>().sharedMesh = mesh;

                    JUtil.LogMessage(null, "PROP_BATCH: batching {0} transforms under {1}", batchList.Count, batchList[0].transform.parent.name);
                }
            }

            JUtil.LogMessage(null, "PROP_BATCH: old prop count: {0}; new prop count: {1}; delta {2}; total batched transforms {3}; removed {4} modules",
                    internalModel.props.Count, newProps.Count, internalModel.props.Count - newProps.Count, batchedTransformCount, removedModuleCount);
            internalModel.props = newProps;
        }
        private void Font_textureRebuilt(Font font)
        {
            JUtil.LogMessage(this, "Font {0} rebuilt", font);

            foreach (var batch in labelBatches.Values)
            {
                if (batch.batchInfo.font == font)
                {
                    batch.Rebuild();
                }
            }
        }

        // Text label batching:
        // A large number of JSILabel modules are completely static - things like labels on switches, buttons, etc.
        // But these can still sometimes change color, especially when the backlight is turned on.
        // For all the JSILabels with the same controlling variable and color settings, we can render them all at once in a single mesh.

        class LabelBatch
        {
            public GameObject batchRoot;
            public MeshRenderer renderer;
            public MeshFilter meshFilter;
            public JSILabel.TextBatchInfo batchInfo;

            public List<JSITextMesh> textMeshes = new List<JSITextMesh>();
            public bool needsUpdate = true;

            public LabelBatch(JSILabel firstLabel)
            {
                batchRoot = new GameObject("Label Batch Root");
                batchRoot.layer = 20;

                renderer = batchRoot.AddComponent<MeshRenderer>();
                renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                renderer.receiveShadows = true;
                renderer.material = firstLabel.textObj.material;
                renderer.material.mainTexture = firstLabel.batchInfo.font.material.mainTexture;
                meshFilter = batchRoot.AddComponent<MeshFilter>();

                batchInfo = firstLabel.batchInfo;

                if (batchInfo.variableName != null)
                {
                    firstLabel.rpmComp.RegisterVariableCallback(batchInfo.variableName, VariableChangedCallback);
                }
            }

            public void VariableChangedCallback(float value)
            {
                Color32 newColor;
                float newEmissive; // TODO: flash support...

                if (value == 0.0f)
                {
                    newColor = batchInfo.zeroColor;
                    newEmissive = 0.0f;
                }
                else if (value < 0.0f)
                {
                    newColor = batchInfo.negativeColor;
                    newEmissive = 0.0f;
                }
                else
                {
                    newColor = batchInfo.positiveColor;
                    newEmissive = 1.0f;
                }

                foreach (var textMesh in textMeshes)
                {
                    if (!newColor.Compare(textMesh.color))
                    {
                        textMesh.color = newColor;
                        needsUpdate = true;
                    }
                }

                if (newEmissive != renderer.sharedMaterial.GetFloat(JSILabel.emissiveFactorIndex))
                {
                    renderer.sharedMaterial.SetFloat(JSILabel.emissiveFactorIndex, newEmissive);
                }
            }

            public void Rebuild()
            {
                needsUpdate = true;
                foreach (var textMesh in textMeshes)
                {
                    textMesh.Invalidate();
                }
            }

            public void LateUpdate()
            {
                if (!needsUpdate) return;
                needsUpdate = false; // must do this early because regenerating the text can invalidate it again!

                var worldToLocal = batchRoot.transform.worldToLocalMatrix;
                CombineInstance[] instances = new CombineInstance[textMeshes.Count];
                for (int i = 0; i < instances.Length; ++i)
                {
                    instances[i] = new CombineInstance();
                    textMeshes[i].Build();
                    instances[i].mesh = textMeshes[i].mesh;
                    instances[i].transform = worldToLocal * textMeshes[i].transform.localToWorldMatrix;
                }

                meshFilter.mesh.Clear();
                CombineMeshes(meshFilter.mesh, instances);
                meshFilter.mesh.UploadMeshData(false);
            }
        }

        readonly Dictionary<JSILabel.TextBatchInfo, LabelBatch> labelBatches = new Dictionary<JSILabel.TextBatchInfo, LabelBatch>();
        RasterPropMonitorComputer rpmComp = null;

        public void AddStaticLabel(JSILabel label)
        {
            LabelBatch labelBatch;
            if (!labelBatches.TryGetValue(label.batchInfo, out labelBatch))
            {
                rpmComp = label.rpmComp;
                labelBatch = new LabelBatch(label);
                labelBatches.Add(label.batchInfo, labelBatch);
                // TODO: hook up flashing behavior

                labelBatch.batchRoot.transform.SetParent(transform, false);
            }

            label.textObj.transform.SetParent(labelBatch.batchRoot.transform, true);
            label.textObj.gameObject.SetActive(false);
            label.textObj.color = label.batchInfo.zeroColor;

            labelBatch.textMeshes.Add(label.textObj);
            labelBatch.needsUpdate = true;

            Component.Destroy(label.textObj.transform.GetComponent<MeshRenderer>());
            // todo: destroy meshfilter? but we need the meshes to stick around.
            label.internalProp.internalModules.Remove(label);
            Component.Destroy(label);
        }

        void Start()
        {
            Font.textureRebuilt += Font_textureRebuilt;
        }

        void LateUpdate()
        {
            foreach (var labelBatch in labelBatches.Values)
            {
                labelBatch.LateUpdate();
            }
        }

        void OnDestroy()
        {
            Font.textureRebuilt -= Font_textureRebuilt;

            if (rpmComp != null)
            {
                foreach (var labelBatch in labelBatches)
                {
                    if (labelBatch.Key.variableName != null)
                    {
                        rpmComp.UnregisterVariableCallback(labelBatch.Key.variableName, labelBatch.Value.VariableChangedCallback);
                    }
                }
            }
        }

        private static void CombineMeshes(Mesh mesh, CombineInstance[] instances)
        {
            // Unity 2019.4 silently truncates indices > 2^16 when combining meshes
            // unless we explicitly set the index format to uint32.
            int vertexCount = 0;
            foreach (var instance in instances)
                vertexCount += instance.mesh.vertexCount;
            
            mesh.Clear();
            mesh.indexFormat = vertexCount <= ushort.MaxValue
                ? IndexFormat.UInt16
                : IndexFormat.UInt32;
            mesh.CombineMeshes(instances);
        }
    }
}
