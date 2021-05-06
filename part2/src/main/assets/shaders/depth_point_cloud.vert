/*
 * Copyright 2021 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

uniform mat4 u_ModelViewProjection;
uniform float u_PointSize;

attribute vec4 a_Position;

varying vec4 v_Color;

// Returns an interpolated color in a 6 degree polynomial interpolation.
vec3 GetPolynomialColor(in float x,
  in vec4 kRedVec4, in vec4 kGreenVec4, in vec4 kBlueVec4,
  in vec2 kRedVec2, in vec2 kGreenVec2, in vec2 kBlueVec2) {
  // Moves the color space a little bit to avoid pure red.
  // Removes this line for more contrast.
  x = clamp(x * 0.9 + 0.03, 0.0, 1.0);
  vec4 v4 = vec4(1.0, x, x * x, x * x * x);
  vec2 v2 = v4.zw * v4.z;
  return vec3(
    dot(v4, kRedVec4) + dot(v2, kRedVec2),
    dot(v4, kGreenVec4) + dot(v2, kGreenVec2),
    dot(v4, kBlueVec4) + dot(v2, kBlueVec2)
  );
}

// Returns a smooth Percept colormap based upon the Turbo colormap.
vec3 PerceptColormap(in float x) {
  const vec4 kRedVec4 = vec4(0.55305649, 3.00913185, -5.46192616, -11.11819092);
  const vec4 kGreenVec4 = vec4(0.16207513, 0.17712472, 15.24091500, -36.50657960);
  const vec4 kBlueVec4 = vec4(-0.05195877, 5.18000081, -30.94853351, 81.96403246);
  const vec2 kRedVec2 = vec2(27.81927491, -14.87899417);
  const vec2 kGreenVec2 = vec2(25.95549545, -5.02738237);
  const vec2 kBlueVec2 = vec2(-86.53476570, 30.23299484);
  const float kInvalidDepthThreshold = 0.01;
  return step(kInvalidDepthThreshold, x) *
         GetPolynomialColor(x, kRedVec4, kGreenVec4, kBlueVec4,
                            kRedVec2, kGreenVec2, kBlueVec2);
}

void main() {
   // Colors the pointcloud by height.
   float kMinHeightMeters = -2.0f;
   float kMaxHeightMeters = 2.0f;
   float normalizedHeight = clamp((a_Position.y - kMinHeightMeters) / (kMaxHeightMeters - kMinHeightMeters), 0.0, 1.0);
   v_Color = vec4(PerceptColormap(normalizedHeight), 1.0);
   gl_Position = u_ModelViewProjection * vec4(a_Position.xyz, 1.0);
   gl_PointSize = u_PointSize;
}