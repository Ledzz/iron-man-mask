#pragma once  // This prevents multiple inclusions

struct float2 {
    float x;
    float y;
};

float fract(float x) {
    return x - floor(x);
}

float mix(float a, float b, float t) {
    return a * (1.0 - t) + b * t;
}

float smoothstep(float edge0, float edge1, float x) {
    x = constrain((x - edge0) / (edge1 - edge0), 0.0, 1.0);
    return x * x * (3.0 - 2.0 * x);
}

// 1D Noise functions
float rand(float n) {
    return fract(sin(n) * 43758.5453123);
}

float noise1D(float p) {
    float fl = floor(p);
    float fc = fract(p);
    return mix(rand(fl), rand(fl + 1.0), fc);
}

// 2D Noise function
float noise2D(float x, float y) {
    const float d0 = 0.0;
    const float d1 = 1.0;
    
    float2 b = {floor(x), floor(y)};
    float2 f = {
        smoothstep(0.0, 1.0, fract(x)),
        smoothstep(0.0, 1.0, fract(y))
    };
    
    // Fixed mixing - now using both x and y coordinates
    float mixA = mix(rand(b.x + b.y * 12.345), rand(b.x + d1 + b.y * 12.345), f.x);
    float mixB = mix(rand(b.x + (b.y + d1) * 12.345), rand(b.x + d1 + (b.y + d1) * 12.345), f.x);
    
    return mix(mixA, mixB, f.y);
}