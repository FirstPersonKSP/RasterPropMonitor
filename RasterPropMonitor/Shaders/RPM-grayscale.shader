// Grayscale shader for rentex
Shader "RPM/Grayscale" 
{
	Properties 
	{
		_MainTex ("Render Input", 2D) = "white" {}
		_Gain ("Gain", float) = 1.0
	}
	SubShader {
		ZTest Always Cull Off ZWrite Off Fog { Mode Off }
		Pass 
		{
			CGPROGRAM
				#pragma vertex vert_img
				#pragma fragment frag
				#include "UnityCG.cginc"
			
				sampler2D _MainTex;
				uniform float _Gain;
			
				float4 frag(v2f_img IN) : COLOR 
				{
					float4 c = tex2D (_MainTex, IN.uv);
					
					// CIE 1931 conversion of linear color to luminance
					float Y = c.r * 0.2126 + c.g * 0.7152 + c.b * 0.0722;
					// Apply gain
					Y = saturate(Y * _Gain);
					return half4(Y, Y, Y, c.a);
				}
			ENDCG
		}
	}
}
