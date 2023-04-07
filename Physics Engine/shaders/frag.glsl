#version 330 core
out vec4 FragColor;

in VS_OUT {
  vec2 TexCoord;
  vec3 PosCoord;
  vec4 WorldPosCoord;
  vec4 LightPosCoord;
} vsIn;
// texture sampler
uniform sampler2D texture1;
uniform sampler2D shadowTexture;

float ShadowCalculation(vec4 fragPosLightSpace)
{
    // perform perspective divide
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
    // get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
    float closestDepth = texture(shadowTexture, projCoords.xy).r; 
    // get depth of current fragment from light's perspective
    float currentDepth = projCoords.z;
    // calculate bias (based on depth map resolution and slope)
    float bias = 0.0001;
    // check whether current frag pos is in shadow
    // float shadow = currentDepth - bias > closestDepth  ? 1.0 : 0.0;
    // PCF
    float shadow = 0.0;
    float accum = 0.0;
    vec2 texelSize = 0.25 / textureSize(shadowTexture, 0);
    for(float x = -4; x <= 4; x += 1)
    {
        for(float y = -4; y <= 4; y += 1)
        {
            float pcfDepth = texture(shadowTexture, projCoords.xy + vec2(x, y) * texelSize).r; 
            shadow += currentDepth - bias > pcfDepth  ? 0.0 : 1.0;
            accum += 1.0;        
        }    
    }
    shadow /= accum;
    
    // keep the shadow at 0.0 when outside the far_plane region of the light's frustum.
    if(projCoords.z > 1.0)
        shadow = 1.0;
        
    return shadow;
}  

void main()
{
    vec3 faceNorm = normalize(cross(dFdx(vsIn.PosCoord), dFdy(vsIn.PosCoord)));
    float lums = dot(faceNorm, vec3(-1, -1, -1)) * ShadowCalculation(vsIn.LightPosCoord) + 0.15;
	FragColor = vec4(texture(texture1, vsIn.TexCoord * 2.0).xyz * lums, 1.0);
}