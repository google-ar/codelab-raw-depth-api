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

package com.google.ar.core.codelab.common.rendering;

import android.content.Context;
import android.opengl.GLES20;
import android.opengl.Matrix;

import com.google.ar.core.Camera;
import com.google.ar.core.codelab.common.helpers.AABB;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;

public class BoxRenderer {
    private static final String TAG = BoxRenderer.class.getSimpleName();
    private static final int NUM_FACES = 6;

    private static final String VERTEX_SHADER_NAME = "shaders/box.vert";
    private static final String FRAGMENT_SHADER_NAME = "shaders/box.frag";

    // Stores the triangulation of the cube.
    private FloatBuffer vertexBuffer;
    private ShortBuffer indexBuffer;

    // Shader program.
    private int program;
    private int vPosition;
    private int uViewProjection;

    public BoxRenderer() {}

    public void createOnGlThread(Context context) throws IOException {
        ShaderUtil.checkGLError(TAG, "Create");
        int vertexShader =
                ShaderUtil.loadGLShader(TAG, context, GLES20.GL_VERTEX_SHADER, VERTEX_SHADER_NAME);
        int fragmentShader =
                ShaderUtil.loadGLShader(TAG, context, GLES20.GL_FRAGMENT_SHADER, FRAGMENT_SHADER_NAME);

        program = GLES20.glCreateProgram();
        GLES20.glAttachShader(program, vertexShader);
        GLES20.glAttachShader(program, fragmentShader);
        GLES20.glLinkProgram(program);
        GLES20.glUseProgram(program);
        ShaderUtil.checkGLError(TAG, "Program");

        vPosition = GLES20.glGetAttribLocation(program, "vPosition");
        uViewProjection = GLES20.glGetUniformLocation(program, "uViewProjection");

        // Sets the index buffer, which is constant regardless of the size/position of the cube.
        short[] indices = {
            0, 1, 2, 2, 1, 3, // Front.
            4, 5, 6, 6, 5, 7, // Back.
            8, 9, 10, 10, 9, 11, // Left.
            12, 13, 14, 14, 13, 15, // Right.
            16, 17, 18, 18, 17, 19, // Top.
            20, 21, 22, 22, 21, 23  // Bottom.
        };
        ByteBuffer indexByteBuffer = ByteBuffer.allocateDirect(2 * indices.length); // 2 bytes per short.
        indexByteBuffer.order(ByteOrder.nativeOrder());
        indexBuffer = indexByteBuffer.asShortBuffer();
        indexBuffer.rewind();
        indexBuffer.put(indices);

        // Allocates vertexBuffer.
        ByteBuffer vertexByteBuffer = ByteBuffer.allocateDirect(288); // 6 faces * 4 corners * 3 dimensions * 4 bytes-per-float.
        vertexByteBuffer.order(ByteOrder.nativeOrder());
        vertexBuffer = vertexByteBuffer.asFloatBuffer();
        ShaderUtil.checkGLError(TAG, "Init complete");
    }

    private void setCubeDimensions(AABB aabb) {
        float[] vertices = {
                // Front.
                aabb.minX, aabb.minY, aabb.maxZ,
                aabb.maxX, aabb.minY, aabb.maxZ,
                aabb.minX, aabb.maxY, aabb.maxZ,
                aabb.maxX, aabb.maxY, aabb.maxZ,
                // Back.
                aabb.maxX, aabb.minY, aabb.minZ,
                aabb.minX, aabb.minY, aabb.minZ,
                aabb.maxX, aabb.maxY, aabb.minZ,
                aabb.minX, aabb.maxY, aabb.minZ,
                // Left.
                aabb.minX, aabb.minY, aabb.minZ,
                aabb.minX, aabb.minY, aabb.maxZ,
                aabb.minX, aabb.maxY, aabb.minZ,
                aabb.minX, aabb.maxY, aabb.maxZ,
                // Right.
                aabb.maxX, aabb.minY, aabb.maxZ,
                aabb.maxX, aabb.minY, aabb.minZ,
                aabb.maxX, aabb.maxY, aabb.maxZ,
                aabb.maxX, aabb.maxY, aabb.minZ,
                // Top.
                aabb.minX, aabb.maxY, aabb.maxZ,
                aabb.maxX, aabb.maxY, aabb.maxZ,
                aabb.minX, aabb.maxY, aabb.minZ,
                aabb.maxX, aabb.maxY, aabb.minZ,
                // Bottom.
                aabb.minX, aabb.minY, aabb.minZ,
                aabb.maxX, aabb.minY, aabb.minZ,
                aabb.minX, aabb.minY, aabb.maxZ,
                aabb.maxX, aabb.minY, aabb.maxZ
        };
        vertexBuffer.rewind();
        vertexBuffer.put(vertices);
    }

    public void draw(AABB aabb, Camera camera) {
        float[] projectionMatrix = new float[16];
        camera.getProjectionMatrix(projectionMatrix, 0, 0.1f, 100.0f);
        float[] viewMatrix = new float[16];
        camera.getViewMatrix(viewMatrix, 0);
        float[] viewProjection = new float[16];
        Matrix.multiplyMM(viewProjection, 0, projectionMatrix, 0, viewMatrix, 0);

        // Updates the positions of the cube.
        setCubeDimensions(aabb);
        GLES20.glUseProgram(program);
        GLES20.glUniformMatrix4fv(uViewProjection, 1, false, viewProjection, 0);
        GLES20.glEnableVertexAttribArray(vPosition);
        vertexBuffer.rewind();
        GLES20.glVertexAttribPointer(vPosition, 3,
                GLES20.GL_FLOAT, false,
                12, vertexBuffer);
        ShaderUtil.checkGLError(TAG, "Draw");

        // Draws a cube.
        GLES20.glEnable(GLES20.GL_BLEND);
        GLES20.glBlendFunc(GLES20.GL_SRC_ALPHA, GLES20.GL_ONE_MINUS_SRC_ALPHA);
        indexBuffer.rewind();
        GLES20.glDrawElements(GLES20.GL_TRIANGLES, indexBuffer.remaining(), GLES20.GL_UNSIGNED_SHORT, indexBuffer);
        GLES20.glDisableVertexAttribArray(vPosition);
        ShaderUtil.checkGLError(TAG, "Draw complete");
    }
}
