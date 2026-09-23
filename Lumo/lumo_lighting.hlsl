Texture2D		g_texture0 : register(t0);
SamplerState	g_sampler0 : register(s0);

namespace s3d
{
	struct PSInput
	{
		float4 position	: SV_POSITION;
		float4 color	: COLOR0;
		float2 uv		: TEXCOORD0;
	};
}

cbuffer PSConstants2D : register(b0)
{
	float4 g_colorAdd;
	float4 g_sdfParam;
	float4 g_sdfOutlineColor;
	float4 g_sdfShadowColor;
	float4 g_internal;
}

cbuffer Resource : register(b1)
{
	float3 g_lightColor;
	float3 g_lightBase;
	float3 g_lightDirection;
}

float4 PS(s3d::PSInput input) : SV_TARGET
{
	float2 uv = input.uv;
	float4 tex = g_texture0.Sample(g_sampler0, uv);
	float3 normal = tex.rgb * 2.0f - 1.0f;
	float alpha = tex.a;

	float4 color = float4(saturate(dot(normal, g_lightDirection)) * g_lightColor, alpha);
	color.rgb += g_lightBase;
	return (color * input.color) + g_colorAdd;
}
