cbuffer SceneConstantBuffer : register(b0)
{
    matrix model;
    matrix view;
    matrix projection;
    float4 ambientColor;
    float4 directionalLightDir;
    float4 directionalLightColor;
};

struct PSInput
{
    float4 position : SV_POSITION;
    float4 color : COLOR;
    float3 normal : NORMAL;
    float3 worldPos : TEXCOORD0;
};

PSInput VSMain(float3 position : POSITION, float3 normal : NORMAL, float4 color : COLOR)
{
    PSInput result;

    // Transform position to world space
    float4 worldPos = mul(model, float4(position, 1.0f));
    result.worldPos = worldPos.xyz;

    // Transform normal to world space (assuming uniform scaling)
    result.normal = normalize(mul((float3x3)model, normal));

    // View projection
    result.position = mul(view, worldPos);
    result.position = mul(projection, result.position);

    result.color = color;
    return result;
}

float4 PSMain(PSInput input) : SV_TARGET
{
    // Basic directional lighting
    float3 normal = normalize(input.normal);
    float3 lightDir = normalize(-directionalLightDir.xyz);
    
    // Ambient
    float3 ambient = ambientColor.rgb * ambientColor.a;
    
    // Diffuse
    float NdotL = max(dot(normal, lightDir), 0.0f);
    float3 diffuse = NdotL * directionalLightColor.rgb * directionalLightColor.a;
    
    // Combine
    float3 finalColor = input.color.rgb * (ambient + diffuse);
    
    // Hacky handling for unlit lines (if normal length is basically 0, it's a debug line)
    if (dot(input.normal, input.normal) < 0.01f) {
        return float4(input.color.rgb, 1.0f);
    }

    return float4(finalColor, input.color.a);
}
