/*
 * Copyright 2021 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.google.ar.core.codelab.common.helpers;

// An axis-aligned bounding box is defined by the minimum and maximum extends in each dimension.
public class AABB {
    public float minX = Float.MAX_VALUE;
    public float minY = Float.MAX_VALUE;
    public float minZ = Float.MAX_VALUE;
    public float maxX = -Float.MAX_VALUE;
    public float maxY = -Float.MAX_VALUE;
    public float maxZ = -Float.MAX_VALUE;

    public void update(float x, float y, float z) {
        minX = Math.min(x, minX);
        minY = Math.min(y, minY);
        minZ = Math.min(z, minZ);
        maxX = Math.max(x, maxX);
        maxY = Math.max(y, maxY);
        maxZ = Math.max(z, maxZ);
    }
}
