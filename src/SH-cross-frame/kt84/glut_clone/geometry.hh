#pragma once
#include "../util.hh"
#include <GL/glew.h>
#include <iostream>
#include <vector>
// mostly copied from freeglut_geometry.c

namespace kt84 {
    namespace internal {
        inline void fghCircleTable(double **sint, double **cost, const int n)
        {
            int i;

            /* Table size, the sign of n flips the circle direction */

            const int size = abs(n);

            /* Determine the angle between samples */

            const double angle = 2 * util::pi() / (double)((n == 0) ? 1 : n);

            /* Allocate memory for n samples, plus duplicate of first entry at the end */

            *sint = (double *)calloc(sizeof(double), size + 1);
            *cost = (double *)calloc(sizeof(double), size + 1);

            /* Bail out if memory allocation fails, fgError never returns */

            if (!(*sint) || !(*cost))
            {
                free(*sint);
                free(*cost);
                std::cerr << "Failed to allocate memory in fghCircleTable";
            }

            /* Compute cos and sin around the circle */

            (*sint)[0] = 0.0;
            (*cost)[0] = 1.0;

            for (i = 1; i < size; i++)
            {
                (*sint)[i] = sin(angle*i);
                (*cost)[i] = cos(angle*i);
            }

            /* Last sample is duplicate of the first */

            (*sint)[size] = (*sint)[0];
            (*cost)[size] = (*cost)[0];
        }
        inline GLdouble** tet_r() {
            static std::vector<std::vector<GLdouble>> buf1 = { { 1.0, 0.0, 0.0 },
            { -0.333333333333, 0.942809041582, 0.0 },
            { -0.333333333333, -0.471404520791, 0.816496580928 },
            { -0.333333333333, -0.471404520791, -0.816496580928 } };
            static std::vector<GLdouble*> buf2 = { &buf1[0][0], &buf1[1][0], &buf1[2][0], &buf1[3][0] };
            return &buf2[0];
        }
        inline GLint** tet_i() {
            static std::vector<std::vector<GLint>> buf1 =  /* Vertex indices */
            {
                { 1, 3, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 0, 1, 2 }
            };
            static std::vector<GLint*> buf2 = { &buf1[0][0], &buf1[1][0], &buf1[2][0], &buf1[3][0] };
            return &buf2[0];
        }
        inline double** icos_r() {
            static std::vector<std::vector<double>> buf1 = {
                { 1.0, 0.0, 0.0 },
                { 0.447213595500, 0.894427191000, 0.0 },
                { 0.447213595500, 0.276393202252, 0.850650808354 },
                { 0.447213595500, -0.723606797748, 0.525731112119 },
                { 0.447213595500, -0.723606797748, -0.525731112119 },
                { 0.447213595500, 0.276393202252, -0.850650808354 },
                { -0.447213595500, -0.894427191000, 0.0 },
                { -0.447213595500, -0.276393202252, 0.850650808354 },
                { -0.447213595500, 0.723606797748, 0.525731112119 },
                { -0.447213595500, 0.723606797748, -0.525731112119 },
                { -0.447213595500, -0.276393202252, -0.850650808354 },
                { -1.0, 0.0, 0.0 }
            };
            static std::vector<double*> buf2 = {
                &buf1[0][0],
                &buf1[1][0],
                &buf1[2][0],
                &buf1[3][0],
                &buf1[4][0],
                &buf1[5][0],
                &buf1[6][0],
                &buf1[7][0],
                &buf1[8][0],
                &buf1[9][0],
                &buf1[10][0],
                &buf1[11][0]
            };
            return &buf2[0];
        }
        inline int** icos_v() {
            static std::vector<std::vector<int>> buf1 = {
                { 0, 1, 2 },
                { 0, 2, 3 },
                { 0, 3, 4 },
                { 0, 4, 5 },
                { 0, 5, 1 },
                { 1, 8, 2 },
                { 2, 7, 3 },
                { 3, 6, 4 },
                { 4, 10, 5 },
                { 5, 9, 1 },
                { 1, 9, 8 },
                { 2, 8, 7 },
                { 3, 7, 6 },
                { 4, 6, 10 },
                { 5, 10, 9 },
                { 11, 9, 10 },
                { 11, 8, 9 },
                { 11, 7, 8 },
                { 11, 6, 7 },
                { 11, 10, 6 }
            };
            static std::vector<int*> buf2 = {
                &buf1[0][0],
                &buf1[1][0],
                &buf1[2][0],
                &buf1[3][0],
                &buf1[4][0],
                &buf1[5][0],
                &buf1[6][0],
                &buf1[7][0],
                &buf1[8][0],
                &buf1[9][0],
                &buf1[10][0],
                &buf1[11][0],
                &buf1[12][0],
                &buf1[13][0],
                &buf1[14][0],
                &buf1[15][0],
                &buf1[16][0],
                &buf1[17][0],
                &buf1[18][0],
                &buf1[19][0],
            };
            return &buf2[0];
        }
        inline double** rdod_r() {
            static std::vector<std::vector<double>> buf1 = {
                { 0.0, 0.0, 1.0 },
                { 0.707106781187, 0.000000000000, 0.5 },
                { 0.000000000000, 0.707106781187, 0.5 },
                { -0.707106781187, 0.000000000000, 0.5 },
                { 0.000000000000, -0.707106781187, 0.5 },
                { 0.707106781187, 0.707106781187, 0.0 },
                { -0.707106781187, 0.707106781187, 0.0 },
                { -0.707106781187, -0.707106781187, 0.0 },
                { 0.707106781187, -0.707106781187, 0.0 },
                { 0.707106781187, 0.000000000000, -0.5 },
                { 0.000000000000, 0.707106781187, -0.5 },
                { -0.707106781187, 0.000000000000, -0.5 },
                { 0.000000000000, -0.707106781187, -0.5 },
                { 0.0, 0.0, -1.0 }
            };
            static std::vector<double*> buf2 = {
                &buf1[0][0],
                &buf1[1][0],
                &buf1[2][0],
                &buf1[3][0],
                &buf1[4][0],
                &buf1[5][0],
                &buf1[6][0],
                &buf1[7][0],
                &buf1[8][0],
                &buf1[9][0],
                &buf1[10][0],
                &buf1[11][0],
                &buf1[12][0],
                &buf1[13][0],
            };
            return &buf2[0];
        }
        inline int** rdod_v() {
            static std::vector<std::vector<int>> buf1 = {
                { 0, 1, 5, 2 },
                { 0, 2, 6, 3 },
                { 0, 3, 7, 4 },
                { 0, 4, 8, 1 },
                { 5, 10, 6, 2 },
                { 6, 11, 7, 3 },
                { 7, 12, 8, 4 },
                { 8, 9, 5, 1 },
                { 5, 9, 13, 10 },
                { 6, 10, 13, 11 },
                { 7, 11, 13, 12 },
                { 8, 12, 13, 9 }
            };
            static std::vector<int*> buf2 = {
                &buf1[0][0],
                &buf1[1][0],
                &buf1[2][0],
                &buf1[3][0],
                &buf1[4][0],
                &buf1[5][0],
                &buf1[6][0],
                &buf1[7][0],
                &buf1[8][0],
                &buf1[9][0],
                &buf1[10][0],
                &buf1[11][0],
            };
            return &buf2[0];
        }
        inline double** rdod_n() {
            static std::vector<std::vector<double>> buf1 = {
                { 0.353553390594, 0.353553390594, 0.5 },
                { -0.353553390594, 0.353553390594, 0.5 },
                { -0.353553390594, -0.353553390594, 0.5 },
                { 0.353553390594, -0.353553390594, 0.5 },
                { 0.000000000000, 1.000000000000, 0.0 },
                { -1.000000000000, 0.000000000000, 0.0 },
                { 0.000000000000, -1.000000000000, 0.0 },
                { 1.000000000000, 0.000000000000, 0.0 },
                { 0.353553390594, 0.353553390594, -0.5 },
                { -0.353553390594, 0.353553390594, -0.5 },
                { -0.353553390594, -0.353553390594, -0.5 },
                { 0.353553390594, -0.353553390594, -0.5 }
            };
            static std::vector<double*> buf2 = {
                &buf1[0][0],
                &buf1[1][0],
                &buf1[2][0],
                &buf1[3][0],
                &buf1[4][0],
                &buf1[5][0],
                &buf1[6][0],
                &buf1[7][0],
                &buf1[8][0],
                &buf1[9][0],
                &buf1[10][0],
                &buf1[11][0],
            };
            return &buf2[0];
        }
    }

    inline void glutWireCube(GLdouble dSize)
    {
        double size = dSize * 0.5;

#   define V(a,b,c) glVertex3d( a size, b size, c size );
#   define N(a,b,c) glNormal3d( a, b, c );

        /* PWO: I dared to convert the code to use macros... */
        glBegin(GL_LINE_LOOP); N(1.0, 0.0, 0.0); V(+, -, +); V(+, -, -); V(+, +, -); V(+, +, +); glEnd();
        glBegin(GL_LINE_LOOP); N(0.0, 1.0, 0.0); V(+, +, +); V(+, +, -); V(-, +, -); V(-, +, +); glEnd();
        glBegin(GL_LINE_LOOP); N(0.0, 0.0, 1.0); V(+, +, +); V(-, +, +); V(-, -, +); V(+, -, +); glEnd();
        glBegin(GL_LINE_LOOP); N(-1.0, 0.0, 0.0); V(-, -, +); V(-, +, +); V(-, +, -); V(-, -, -); glEnd();
        glBegin(GL_LINE_LOOP); N(0.0, -1.0, 0.0); V(-, -, +); V(-, -, -); V(+, -, -); V(+, -, +); glEnd();
        glBegin(GL_LINE_LOOP); N(0.0, 0.0, -1.0); V(-, -, -); V(-, +, -); V(+, +, -); V(+, -, -); glEnd();

#   undef V
#   undef N
    }
    inline void glutSolidCube(GLdouble dSize)
    {
        double size = dSize * 0.5;

#   define V(a,b,c) glVertex3d( a size, b size, c size );
#   define N(a,b,c) glNormal3d( a, b, c );

        /* PWO: Again, I dared to convert the code to use macros... */
        glBegin(GL_QUADS);
        N(1.0, 0.0, 0.0); V(+, -, +); V(+, -, -); V(+, +, -); V(+, +, +);
        N(0.0, 1.0, 0.0); V(+, +, +); V(+, +, -); V(-, +, -); V(-, +, +);
        N(0.0, 0.0, 1.0); V(+, +, +); V(-, +, +); V(-, -, +); V(+, -, +);
        N(-1.0, 0.0, 0.0); V(-, -, +); V(-, +, +); V(-, +, -); V(-, -, -);
        N(0.0, -1.0, 0.0); V(-, -, +); V(-, -, -); V(+, -, -); V(+, -, +);
        N(0.0, 0.0, -1.0); V(-, -, -); V(-, +, -); V(+, +, -); V(+, -, -);
        glEnd();

#   undef V
#   undef N
    }
    inline void glutSolidSphere(GLdouble radius, GLint slices, GLint stacks)
    {
        int i, j;

        /* Adjust z and radius as stacks are drawn. */

        double z0, z1;
        double r0, r1;

        /* Pre-computed circle */

        double *sint1, *cost1;
        double *sint2, *cost2;

        internal::fghCircleTable(&sint1, &cost1, -slices);
        internal::fghCircleTable(&sint2, &cost2, stacks * 2);

        /* The top stack is covered with a triangle fan */

        z0 = 1.0;
        z1 = cost2[(stacks > 0) ? 1 : 0];
        r0 = 0.0;
        r1 = sint2[(stacks > 0) ? 1 : 0];

        glBegin(GL_TRIANGLE_FAN);

        glNormal3d(0, 0, 1);
        glVertex3d(0, 0, radius);

        for (j = slices; j >= 0; j--)
        {
            glNormal3d(cost1[j] * r1, sint1[j] * r1, z1);
            glVertex3d(cost1[j] * r1*radius, sint1[j] * r1*radius, z1*radius);
        }

        glEnd();

        /* Cover each stack with a quad strip, except the top and bottom stacks */

        for (i = 1; i < stacks - 1; i++)
        {
            z0 = z1; z1 = cost2[i + 1];
            r0 = r1; r1 = sint2[i + 1];

            glBegin(GL_QUAD_STRIP);

            for (j = 0; j <= slices; j++)
            {
                glNormal3d(cost1[j] * r1, sint1[j] * r1, z1);
                glVertex3d(cost1[j] * r1*radius, sint1[j] * r1*radius, z1*radius);
                glNormal3d(cost1[j] * r0, sint1[j] * r0, z0);
                glVertex3d(cost1[j] * r0*radius, sint1[j] * r0*radius, z0*radius);
            }

            glEnd();
        }

        /* The bottom stack is covered with a triangle fan */

        z0 = z1;
        r0 = r1;

        glBegin(GL_TRIANGLE_FAN);

        glNormal3d(0, 0, -1);
        glVertex3d(0, 0, -radius);

        for (j = 0; j <= slices; j++)
        {
            glNormal3d(cost1[j] * r0, sint1[j] * r0, z0);
            glVertex3d(cost1[j] * r0*radius, sint1[j] * r0*radius, z0*radius);
        }

        glEnd();

        /* Release sin and cos tables */

        free(sint1);
        free(cost1);
        free(sint2);
        free(cost2);
    }
    inline void glutWireSphere(GLdouble radius, GLint slices, GLint stacks)
    {
        int i, j;

        /* Adjust z and radius as stacks and slices are drawn. */

        double r;
        double x, y, z;

        /* Pre-computed circle */

        double *sint1, *cost1;
        double *sint2, *cost2;

        internal::fghCircleTable(&sint1, &cost1, -slices);
        internal::fghCircleTable(&sint2, &cost2, stacks * 2);

        /* Draw a line loop for each stack */

        for (i = 1; i < stacks; i++)
        {
            z = cost2[i];
            r = sint2[i];

            glBegin(GL_LINE_LOOP);

            for (j = 0; j <= slices; j++)
            {
                x = cost1[j];
                y = sint1[j];

                glNormal3d(x, y, z);
                glVertex3d(x*r*radius, y*r*radius, z*radius);
            }

            glEnd();
        }

        /* Draw a line loop for each slice */

        for (i = 0; i < slices; i++)
        {
            glBegin(GL_LINE_STRIP);

            for (j = 0; j <= stacks; j++)
            {
                x = cost1[i] * sint2[j];
                y = sint1[i] * sint2[j];
                z = cost2[j];

                glNormal3d(x, y, z);
                glVertex3d(x*radius, y*radius, z*radius);
            }

            glEnd();
        }

        /* Release sin and cos tables */

        free(sint1);
        free(cost1);
        free(sint2);
        free(cost2);
    }
    inline void glutSolidCone(GLdouble base, GLdouble height, GLint slices, GLint stacks)
    {
        int i, j;

        /* Step in z and radius as stacks are drawn. */

        double z0, z1;
        double r0, r1;

        const double zStep = height / ((stacks > 0) ? stacks : 1);
        const double rStep = base / ((stacks > 0) ? stacks : 1);

        /* Scaling factors for vertex normals */

        const double cosn = (height / sqrt(height * height + base * base));
        const double sinn = (base / sqrt(height * height + base * base));

        /* Pre-computed circle */

        double *sint, *cost;

        internal::fghCircleTable(&sint, &cost, -slices);

        /* Cover the circular base with a triangle fan... */

        z0 = 0.0;
        z1 = zStep;

        r0 = base;
        r1 = r0 - rStep;

        glBegin(GL_TRIANGLE_FAN);

        glNormal3d(0.0, 0.0, -1.0);
        glVertex3d(0.0, 0.0, z0);

        for (j = 0; j <= slices; j++)
            glVertex3d(cost[j] * r0, sint[j] * r0, z0);

        glEnd();

        /* Cover each stack with a quad strip, except the top stack */

        for (i = 0; i < stacks - 1; i++)
        {
            glBegin(GL_QUAD_STRIP);

            for (j = 0; j <= slices; j++)
            {
                glNormal3d(cost[j] * cosn, sint[j] * cosn, sinn);
                glVertex3d(cost[j] * r0, sint[j] * r0, z0);
                glVertex3d(cost[j] * r1, sint[j] * r1, z1);
            }

            z0 = z1; z1 += zStep;
            r0 = r1; r1 -= rStep;

            glEnd();
        }

        /* The top stack is covered with individual triangles */

        glBegin(GL_TRIANGLES);

        glNormal3d(cost[0] * cosn, sint[0] * cosn, sinn);

        for (j = 0; j < slices; j++)
        {
            glVertex3d(cost[j + 0] * r0, sint[j + 0] * r0, z0);
            glVertex3d(0, 0, height);
            glNormal3d(cost[j + 1] * cosn, sint[j + 1] * cosn, sinn);
            glVertex3d(cost[j + 1] * r0, sint[j + 1] * r0, z0);
        }

        glEnd();

        /* Release sin and cos tables */

        free(sint);
        free(cost);
    }
    inline void glutWireCone(GLdouble base, GLdouble height, GLint slices, GLint stacks)
    {
        int i, j;

        /* Step in z and radius as stacks are drawn. */

        double z = 0.0;
        double r = base;

        const double zStep = height / ((stacks > 0) ? stacks : 1);
        const double rStep = base / ((stacks > 0) ? stacks : 1);

        /* Scaling factors for vertex normals */

        const double cosn = (height / sqrt(height * height + base * base));
        const double sinn = (base / sqrt(height * height + base * base));

        /* Pre-computed circle */

        double *sint, *cost;

        internal::fghCircleTable(&sint, &cost, -slices);

        /* Draw the stacks... */

        for (i = 0; i < stacks; i++)
        {
            glBegin(GL_LINE_LOOP);

            for (j = 0; j < slices; j++)
            {
                glNormal3d(cost[j] * sinn, sint[j] * sinn, cosn);
                glVertex3d(cost[j] * r, sint[j] * r, z);
            }

            glEnd();

            z += zStep;
            r -= rStep;
        }

        /* Draw the slices */

        r = base;

        glBegin(GL_LINES);

        for (j = 0; j < slices; j++)
        {
            glNormal3d(cost[j] * cosn, sint[j] * cosn, sinn);
            glVertex3d(cost[j] * r, sint[j] * r, 0.0);
            glVertex3d(0.0, 0.0, height);
        }

        glEnd();

        /* Release sin and cos tables */

        free(sint);
        free(cost);
    }
    inline void glutSolidCylinder(GLdouble radius, GLdouble height, GLint slices, GLint stacks)
    {
        int i, j;

        /* Step in z and radius as stacks are drawn. */

        double z0, z1;
        const double zStep = height / ((stacks > 0) ? stacks : 1);

        /* Pre-computed circle */

        double *sint, *cost;

        internal::fghCircleTable(&sint, &cost, -slices);

        /* Cover the base and top */

        glBegin(GL_TRIANGLE_FAN);
        glNormal3d(0.0, 0.0, -1.0);
        glVertex3d(0.0, 0.0, 0.0);
        for (j = 0; j <= slices; j++)
            glVertex3d(cost[j] * radius, sint[j] * radius, 0.0);
        glEnd();

        glBegin(GL_TRIANGLE_FAN);
        glNormal3d(0.0, 0.0, 1.0);
        glVertex3d(0.0, 0.0, height);
        for (j = slices; j >= 0; j--)
            glVertex3d(cost[j] * radius, sint[j] * radius, height);
        glEnd();

        /* Do the stacks */

        z0 = 0.0;
        z1 = zStep;

        for (i = 1; i <= stacks; i++)
        {
            if (i == stacks)
                z1 = height;

            glBegin(GL_QUAD_STRIP);
            for (j = 0; j <= slices; j++)
            {
                glNormal3d(cost[j], sint[j], 0.0);
                glVertex3d(cost[j] * radius, sint[j] * radius, z0);
                glVertex3d(cost[j] * radius, sint[j] * radius, z1);
            }
            glEnd();

            z0 = z1; z1 += zStep;
        }

        /* Release sin and cos tables */

        free(sint);
        free(cost);
    }
    inline void glutWireCylinder(GLdouble radius, GLdouble height, GLint slices, GLint stacks)
    {
        int i, j;

        /* Step in z and radius as stacks are drawn. */

        double z = 0.0;
        const double zStep = height / ((stacks > 0) ? stacks : 1);

        /* Pre-computed circle */

        double *sint, *cost;

        internal::fghCircleTable(&sint, &cost, -slices);

        /* Draw the stacks... */

        for (i = 0; i <= stacks; i++)
        {
            if (i == stacks)
                z = height;

            glBegin(GL_LINE_LOOP);

            for (j = 0; j < slices; j++)
            {
                glNormal3d(cost[j], sint[j], 0.0);
                glVertex3d(cost[j] * radius, sint[j] * radius, z);
            }

            glEnd();

            z += zStep;
        }

        /* Draw the slices */

        glBegin(GL_LINES);

        for (j = 0; j < slices; j++)
        {
            glNormal3d(cost[j], sint[j], 0.0);
            glVertex3d(cost[j] * radius, sint[j] * radius, 0.0);
            glVertex3d(cost[j] * radius, sint[j] * radius, height);
        }

        glEnd();

        /* Release sin and cos tables */

        free(sint);
        free(cost);
    }
    inline void glutWireTorus(GLdouble dInnerRadius, GLdouble dOuterRadius, GLint nSides, GLint nRings)
    {
        double  iradius = dInnerRadius, oradius = dOuterRadius, phi, psi, dpsi, dphi;
        double *vertex, *normal;
        int    i, j;
        double spsi, cpsi, sphi, cphi;

        if (nSides < 1) nSides = 1;
        if (nRings < 1) nRings = 1;

        /* Allocate the vertices array */
        vertex = (double *)calloc(sizeof(double), 3 * nSides * nRings);
        normal = (double *)calloc(sizeof(double), 3 * nSides * nRings);

        glPushMatrix();

        dpsi = 2.0 * util::pi() / (double)nRings;
        dphi = -2.0 * util::pi() / (double)nSides;
        psi = 0.0;

        for (j = 0; j < nRings; j++)
        {
            cpsi = cos(psi);
            spsi = sin(psi);
            phi = 0.0;

            for (i = 0; i < nSides; i++)
            {
                int offset = 3 * (j * nSides + i);
                cphi = cos(phi);
                sphi = sin(phi);
                *(vertex + offset + 0) = cpsi * (oradius + cphi * iradius);
                *(vertex + offset + 1) = spsi * (oradius + cphi * iradius);
                *(vertex + offset + 2) = sphi * iradius;
                *(normal + offset + 0) = cpsi * cphi;
                *(normal + offset + 1) = spsi * cphi;
                *(normal + offset + 2) = sphi;
                phi += dphi;
            }

            psi += dpsi;
        }

        for (i = 0; i < nSides; i++)
        {
            glBegin(GL_LINE_LOOP);

            for (j = 0; j < nRings; j++)
            {
                int offset = 3 * (j * nSides + i);
                glNormal3dv(normal + offset);
                glVertex3dv(vertex + offset);
            }

            glEnd();
        }

        for (j = 0; j < nRings; j++)
        {
            glBegin(GL_LINE_LOOP);

            for (i = 0; i < nSides; i++)
            {
                int offset = 3 * (j * nSides + i);
                glNormal3dv(normal + offset);
                glVertex3dv(vertex + offset);
            }

            glEnd();
        }

        free(vertex);
        free(normal);
        glPopMatrix();
    }
    inline void glutSolidTorus(GLdouble dInnerRadius, GLdouble dOuterRadius, GLint nSides, GLint nRings)
    {
        double  iradius = dInnerRadius, oradius = dOuterRadius, phi, psi, dpsi, dphi;
        double *vertex, *normal;
        int    i, j;
        double spsi, cpsi, sphi, cphi;

        if (nSides < 1) nSides = 1;
        if (nRings < 1) nRings = 1;

        /* Increment the number of sides and rings to allow for one more point than surface */
        nSides++;
        nRings++;

        /* Allocate the vertices array */
        vertex = (double *)calloc(sizeof(double), 3 * nSides * nRings);
        normal = (double *)calloc(sizeof(double), 3 * nSides * nRings);

        glPushMatrix();

        dpsi = 2.0 * util::pi() / (double)(nRings - 1);
        dphi = -2.0 * util::pi() / (double)(nSides - 1);
        psi = 0.0;

        for (j = 0; j < nRings; j++)
        {
            cpsi = cos(psi);
            spsi = sin(psi);
            phi = 0.0;

            for (i = 0; i < nSides; i++)
            {
                int offset = 3 * (j * nSides + i);
                cphi = cos(phi);
                sphi = sin(phi);
                *(vertex + offset + 0) = cpsi * (oradius + cphi * iradius);
                *(vertex + offset + 1) = spsi * (oradius + cphi * iradius);
                *(vertex + offset + 2) = sphi * iradius;
                *(normal + offset + 0) = cpsi * cphi;
                *(normal + offset + 1) = spsi * cphi;
                *(normal + offset + 2) = sphi;
                phi += dphi;
            }

            psi += dpsi;
        }

        glBegin(GL_QUADS);
        for (i = 0; i < nSides - 1; i++)
        {
            for (j = 0; j < nRings - 1; j++)
            {
                int offset = 3 * (j * nSides + i);
                glNormal3dv(normal + offset);
                glVertex3dv(vertex + offset);
                glNormal3dv(normal + offset + 3);
                glVertex3dv(vertex + offset + 3);
                glNormal3dv(normal + offset + 3 * nSides + 3);
                glVertex3dv(vertex + offset + 3 * nSides + 3);
                glNormal3dv(normal + offset + 3 * nSides);
                glVertex3dv(vertex + offset + 3 * nSides);
            }
        }

        glEnd();

        free(vertex);
        free(normal);
        glPopMatrix();
    }
    inline void glutWireDodecahedron(void)
    {
        /* Magic Numbers:  It is possible to create a dodecahedron by attaching two pentagons to each face of
         * of a cube.  The coordinates of the points are:
         *   (+-x,0, z); (+-1, 1, 1); (0, z, x )
         * where x = (-1 + sqrt(5))/2, z = (1 + sqrt(5))/2  or
         *       x = 0.61803398875 and z = 1.61803398875.
         */
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.0, 0.525731112119, 0.850650808354); glVertex3d(0.0, 1.61803398875, 0.61803398875); glVertex3d(-1.0, 1.0, 1.0); glVertex3d(-0.61803398875, 0.0, 1.61803398875); glVertex3d(0.61803398875, 0.0, 1.61803398875); glVertex3d(1.0, 1.0, 1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.0, 0.525731112119, -0.850650808354); glVertex3d(0.0, 1.61803398875, -0.61803398875); glVertex3d(1.0, 1.0, -1.0); glVertex3d(0.61803398875, 0.0, -1.61803398875); glVertex3d(-0.61803398875, 0.0, -1.61803398875); glVertex3d(-1.0, 1.0, -1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.0, -0.525731112119, 0.850650808354); glVertex3d(0.0, -1.61803398875, 0.61803398875); glVertex3d(1.0, -1.0, 1.0); glVertex3d(0.61803398875, 0.0, 1.61803398875); glVertex3d(-0.61803398875, 0.0, 1.61803398875); glVertex3d(-1.0, -1.0, 1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.0, -0.525731112119, -0.850650808354); glVertex3d(0.0, -1.61803398875, -0.61803398875); glVertex3d(-1.0, -1.0, -1.0); glVertex3d(-0.61803398875, 0.0, -1.61803398875); glVertex3d(0.61803398875, 0.0, -1.61803398875); glVertex3d(1.0, -1.0, -1.0);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glNormal3d(0.850650808354, 0.0, 0.525731112119); glVertex3d(0.61803398875, 0.0, 1.61803398875); glVertex3d(1.0, -1.0, 1.0); glVertex3d(1.61803398875, -0.61803398875, 0.0); glVertex3d(1.61803398875, 0.61803398875, 0.0); glVertex3d(1.0, 1.0, 1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(-0.850650808354, 0.0, 0.525731112119); glVertex3d(-0.61803398875, 0.0, 1.61803398875); glVertex3d(-1.0, 1.0, 1.0); glVertex3d(-1.61803398875, 0.61803398875, 0.0); glVertex3d(-1.61803398875, -0.61803398875, 0.0); glVertex3d(-1.0, -1.0, 1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.850650808354, 0.0, -0.525731112119); glVertex3d(0.61803398875, 0.0, -1.61803398875); glVertex3d(1.0, 1.0, -1.0); glVertex3d(1.61803398875, 0.61803398875, 0.0); glVertex3d(1.61803398875, -0.61803398875, 0.0); glVertex3d(1.0, -1.0, -1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(-0.850650808354, 0.0, -0.525731112119); glVertex3d(-0.61803398875, 0.0, -1.61803398875); glVertex3d(-1.0, -1.0, -1.0); glVertex3d(-1.61803398875, -0.61803398875, 0.0); glVertex3d(-1.61803398875, 0.61803398875, 0.0); glVertex3d(-1.0, 1.0, -1.0);
        glEnd();

        glBegin(GL_LINE_LOOP);
        glNormal3d(0.525731112119, 0.850650808354, 0.0); glVertex3d(1.61803398875, 0.61803398875, 0.0); glVertex3d(1.0, 1.0, -1.0); glVertex3d(0.0, 1.61803398875, -0.61803398875); glVertex3d(0.0, 1.61803398875, 0.61803398875); glVertex3d(1.0, 1.0, 1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.525731112119, -0.850650808354, 0.0); glVertex3d(1.61803398875, -0.61803398875, 0.0); glVertex3d(1.0, -1.0, 1.0); glVertex3d(0.0, -1.61803398875, 0.61803398875); glVertex3d(0.0, -1.61803398875, -0.61803398875); glVertex3d(1.0, -1.0, -1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(-0.525731112119, 0.850650808354, 0.0); glVertex3d(-1.61803398875, 0.61803398875, 0.0); glVertex3d(-1.0, 1.0, 1.0); glVertex3d(0.0, 1.61803398875, 0.61803398875); glVertex3d(0.0, 1.61803398875, -0.61803398875); glVertex3d(-1.0, 1.0, -1.0);
        glEnd();
        glBegin(GL_LINE_LOOP);
        glNormal3d(-0.525731112119, -0.850650808354, 0.0); glVertex3d(-1.61803398875, -0.61803398875, 0.0); glVertex3d(-1.0, -1.0, -1.0); glVertex3d(0.0, -1.61803398875, -0.61803398875); glVertex3d(0.0, -1.61803398875, 0.61803398875); glVertex3d(-1.0, -1.0, 1.0);
        glEnd();
    }
    inline void glutSolidDodecahedron(void)
    {
        /* Magic Numbers:  It is possible to create a dodecahedron by attaching two pentagons to each face of
         * of a cube.  The coordinates of the points are:
         *   (+-x,0, z); (+-1, 1, 1); (0, z, x )
         * where x = (-1 + sqrt(5))/2, z = (1 + sqrt(5))/2 or
         *       x = 0.61803398875 and z = 1.61803398875.
         */
        glBegin(GL_POLYGON);
        glNormal3d(0.0, 0.525731112119, 0.850650808354); glVertex3d(0.0, 1.61803398875, 0.61803398875); glVertex3d(-1.0, 1.0, 1.0); glVertex3d(-0.61803398875, 0.0, 1.61803398875); glVertex3d(0.61803398875, 0.0, 1.61803398875); glVertex3d(1.0, 1.0, 1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(0.0, 0.525731112119, -0.850650808354); glVertex3d(0.0, 1.61803398875, -0.61803398875); glVertex3d(1.0, 1.0, -1.0); glVertex3d(0.61803398875, 0.0, -1.61803398875); glVertex3d(-0.61803398875, 0.0, -1.61803398875); glVertex3d(-1.0, 1.0, -1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(0.0, -0.525731112119, 0.850650808354); glVertex3d(0.0, -1.61803398875, 0.61803398875); glVertex3d(1.0, -1.0, 1.0); glVertex3d(0.61803398875, 0.0, 1.61803398875); glVertex3d(-0.61803398875, 0.0, 1.61803398875); glVertex3d(-1.0, -1.0, 1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(0.0, -0.525731112119, -0.850650808354); glVertex3d(0.0, -1.61803398875, -0.61803398875); glVertex3d(-1.0, -1.0, -1.0); glVertex3d(-0.61803398875, 0.0, -1.61803398875); glVertex3d(0.61803398875, 0.0, -1.61803398875); glVertex3d(1.0, -1.0, -1.0);
        glEnd();

        glBegin(GL_POLYGON);
        glNormal3d(0.850650808354, 0.0, 0.525731112119); glVertex3d(0.61803398875, 0.0, 1.61803398875); glVertex3d(1.0, -1.0, 1.0); glVertex3d(1.61803398875, -0.61803398875, 0.0); glVertex3d(1.61803398875, 0.61803398875, 0.0); glVertex3d(1.0, 1.0, 1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(-0.850650808354, 0.0, 0.525731112119); glVertex3d(-0.61803398875, 0.0, 1.61803398875); glVertex3d(-1.0, 1.0, 1.0); glVertex3d(-1.61803398875, 0.61803398875, 0.0); glVertex3d(-1.61803398875, -0.61803398875, 0.0); glVertex3d(-1.0, -1.0, 1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(0.850650808354, 0.0, -0.525731112119); glVertex3d(0.61803398875, 0.0, -1.61803398875); glVertex3d(1.0, 1.0, -1.0); glVertex3d(1.61803398875, 0.61803398875, 0.0); glVertex3d(1.61803398875, -0.61803398875, 0.0); glVertex3d(1.0, -1.0, -1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(-0.850650808354, 0.0, -0.525731112119); glVertex3d(-0.61803398875, 0.0, -1.61803398875); glVertex3d(-1.0, -1.0, -1.0); glVertex3d(-1.61803398875, -0.61803398875, 0.0); glVertex3d(-1.61803398875, 0.61803398875, 0.0); glVertex3d(-1.0, 1.0, -1.0);
        glEnd();

        glBegin(GL_POLYGON);
        glNormal3d(0.525731112119, 0.850650808354, 0.0); glVertex3d(1.61803398875, 0.61803398875, 0.0); glVertex3d(1.0, 1.0, -1.0); glVertex3d(0.0, 1.61803398875, -0.61803398875); glVertex3d(0.0, 1.61803398875, 0.61803398875); glVertex3d(1.0, 1.0, 1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(0.525731112119, -0.850650808354, 0.0); glVertex3d(1.61803398875, -0.61803398875, 0.0); glVertex3d(1.0, -1.0, 1.0); glVertex3d(0.0, -1.61803398875, 0.61803398875); glVertex3d(0.0, -1.61803398875, -0.61803398875); glVertex3d(1.0, -1.0, -1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(-0.525731112119, 0.850650808354, 0.0); glVertex3d(-1.61803398875, 0.61803398875, 0.0); glVertex3d(-1.0, 1.0, 1.0); glVertex3d(0.0, 1.61803398875, 0.61803398875); glVertex3d(0.0, 1.61803398875, -0.61803398875); glVertex3d(-1.0, 1.0, -1.0);
        glEnd();
        glBegin(GL_POLYGON);
        glNormal3d(-0.525731112119, -0.850650808354, 0.0); glVertex3d(-1.61803398875, -0.61803398875, 0.0); glVertex3d(-1.0, -1.0, -1.0); glVertex3d(0.0, -1.61803398875, -0.61803398875); glVertex3d(0.0, -1.61803398875, 0.61803398875); glVertex3d(-1.0, -1.0, 1.0);
        glEnd();
    }
    inline void glutWireOctahedron(void)
    {
#define RADIUS    1.0f
        glBegin(GL_LINE_LOOP);
        glNormal3d(0.577350269189, 0.577350269189, 0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, RADIUS, 0.0); glVertex3d(0.0, 0.0, RADIUS);
        glNormal3d(0.577350269189, 0.577350269189, -0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, -RADIUS); glVertex3d(0.0, RADIUS, 0.0);
        glNormal3d(0.577350269189, -0.577350269189, 0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, RADIUS); glVertex3d(0.0, -RADIUS, 0.0);
        glNormal3d(0.577350269189, -0.577350269189, -0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, -RADIUS, 0.0); glVertex3d(0.0, 0.0, -RADIUS);
        glNormal3d(-0.577350269189, 0.577350269189, 0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, RADIUS); glVertex3d(0.0, RADIUS, 0.0);
        glNormal3d(-0.577350269189, 0.577350269189, -0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, RADIUS, 0.0); glVertex3d(0.0, 0.0, -RADIUS);
        glNormal3d(-0.577350269189, -0.577350269189, 0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, -RADIUS, 0.0); glVertex3d(0.0, 0.0, RADIUS);
        glNormal3d(-0.577350269189, -0.577350269189, -0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, -RADIUS); glVertex3d(0.0, -RADIUS, 0.0);
        glEnd();
#undef RADIUS
    }
    inline void glutSolidOctahedron(void)
    {
#define RADIUS    1.0f
        glBegin(GL_TRIANGLES);
        glNormal3d(0.577350269189, 0.577350269189, 0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, RADIUS, 0.0); glVertex3d(0.0, 0.0, RADIUS);
        glNormal3d(0.577350269189, 0.577350269189, -0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, -RADIUS); glVertex3d(0.0, RADIUS, 0.0);
        glNormal3d(0.577350269189, -0.577350269189, 0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, RADIUS); glVertex3d(0.0, -RADIUS, 0.0);
        glNormal3d(0.577350269189, -0.577350269189, -0.577350269189); glVertex3d(RADIUS, 0.0, 0.0); glVertex3d(0.0, -RADIUS, 0.0); glVertex3d(0.0, 0.0, -RADIUS);
        glNormal3d(-0.577350269189, 0.577350269189, 0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, RADIUS); glVertex3d(0.0, RADIUS, 0.0);
        glNormal3d(-0.577350269189, 0.577350269189, -0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, RADIUS, 0.0); glVertex3d(0.0, 0.0, -RADIUS);
        glNormal3d(-0.577350269189, -0.577350269189, 0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, -RADIUS, 0.0); glVertex3d(0.0, 0.0, RADIUS);
        glNormal3d(-0.577350269189, -0.577350269189, -0.577350269189); glVertex3d(-RADIUS, 0.0, 0.0); glVertex3d(0.0, 0.0, -RADIUS); glVertex3d(0.0, -RADIUS, 0.0);
        glEnd();
#undef RADIUS
    }
    inline void glutWireTetrahedron(void)
    {
        using namespace internal;
        glBegin(GL_LINE_LOOP);
        glNormal3d(-tet_r()[0][0], -tet_r()[0][1], -tet_r()[0][2]); glVertex3dv(tet_r()[1]); glVertex3dv(tet_r()[3]); glVertex3dv(tet_r()[2]);
        glNormal3d(-tet_r()[1][0], -tet_r()[1][1], -tet_r()[1][2]); glVertex3dv(tet_r()[0]); glVertex3dv(tet_r()[2]); glVertex3dv(tet_r()[3]);
        glNormal3d(-tet_r()[2][0], -tet_r()[2][1], -tet_r()[2][2]); glVertex3dv(tet_r()[0]); glVertex3dv(tet_r()[3]); glVertex3dv(tet_r()[1]);
        glNormal3d(-tet_r()[3][0], -tet_r()[3][1], -tet_r()[3][2]); glVertex3dv(tet_r()[0]); glVertex3dv(tet_r()[1]); glVertex3dv(tet_r()[2]);
        glEnd();
    }
    inline void glutSolidTetrahedron(void)
    {
        using namespace internal;
        glBegin(GL_TRIANGLES);
        glNormal3d(-tet_r()[0][0], -tet_r()[0][1], -tet_r()[0][2]); glVertex3dv(tet_r()[1]); glVertex3dv(tet_r()[3]); glVertex3dv(tet_r()[2]);
        glNormal3d(-tet_r()[1][0], -tet_r()[1][1], -tet_r()[1][2]); glVertex3dv(tet_r()[0]); glVertex3dv(tet_r()[2]); glVertex3dv(tet_r()[3]);
        glNormal3d(-tet_r()[2][0], -tet_r()[2][1], -tet_r()[2][2]); glVertex3dv(tet_r()[0]); glVertex3dv(tet_r()[3]); glVertex3dv(tet_r()[1]);
        glNormal3d(-tet_r()[3][0], -tet_r()[3][1], -tet_r()[3][2]); glVertex3dv(tet_r()[0]); glVertex3dv(tet_r()[1]); glVertex3dv(tet_r()[2]);
        glEnd();
    }
    inline void glutWireIcosahedron(void)
    {
        using namespace internal;
        int i;

        for (i = 0; i < 20; i++)
        {
            double normal[3];
            normal[0] = (icos_r()[icos_v()[i][1]][1] - icos_r()[icos_v()[i][0]][1]) * (icos_r()[icos_v()[i][2]][2] - icos_r()[icos_v()[i][0]][2]) - (icos_r()[icos_v()[i][1]][2] - icos_r()[icos_v()[i][0]][2]) * (icos_r()[icos_v()[i][2]][1] - icos_r()[icos_v()[i][0]][1]);
            normal[1] = (icos_r()[icos_v()[i][1]][2] - icos_r()[icos_v()[i][0]][2]) * (icos_r()[icos_v()[i][2]][0] - icos_r()[icos_v()[i][0]][0]) - (icos_r()[icos_v()[i][1]][0] - icos_r()[icos_v()[i][0]][0]) * (icos_r()[icos_v()[i][2]][2] - icos_r()[icos_v()[i][0]][2]);
            normal[2] = (icos_r()[icos_v()[i][1]][0] - icos_r()[icos_v()[i][0]][0]) * (icos_r()[icos_v()[i][2]][1] - icos_r()[icos_v()[i][0]][1]) - (icos_r()[icos_v()[i][1]][1] - icos_r()[icos_v()[i][0]][1]) * (icos_r()[icos_v()[i][2]][0] - icos_r()[icos_v()[i][0]][0]);
            glBegin(GL_LINE_LOOP);
            glNormal3dv(normal);
            glVertex3dv(icos_r()[icos_v()[i][0]]);
            glVertex3dv(icos_r()[icos_v()[i][1]]);
            glVertex3dv(icos_r()[icos_v()[i][2]]);
            glEnd();
        }
    }
    inline void glutSolidIcosahedron(void)
    {
        using namespace internal;
        int i;

        glBegin(GL_TRIANGLES);
        for (i = 0; i < 20; i++)
        {
            double normal[3];
            normal[0] = (icos_r()[icos_v()[i][1]][1] - icos_r()[icos_v()[i][0]][1]) * (icos_r()[icos_v()[i][2]][2] - icos_r()[icos_v()[i][0]][2]) - (icos_r()[icos_v()[i][1]][2] - icos_r()[icos_v()[i][0]][2]) * (icos_r()[icos_v()[i][2]][1] - icos_r()[icos_v()[i][0]][1]);
            normal[1] = (icos_r()[icos_v()[i][1]][2] - icos_r()[icos_v()[i][0]][2]) * (icos_r()[icos_v()[i][2]][0] - icos_r()[icos_v()[i][0]][0]) - (icos_r()[icos_v()[i][1]][0] - icos_r()[icos_v()[i][0]][0]) * (icos_r()[icos_v()[i][2]][2] - icos_r()[icos_v()[i][0]][2]);
            normal[2] = (icos_r()[icos_v()[i][1]][0] - icos_r()[icos_v()[i][0]][0]) * (icos_r()[icos_v()[i][2]][1] - icos_r()[icos_v()[i][0]][1]) - (icos_r()[icos_v()[i][1]][1] - icos_r()[icos_v()[i][0]][1]) * (icos_r()[icos_v()[i][2]][0] - icos_r()[icos_v()[i][0]][0]);
            glNormal3dv(normal);
            glVertex3dv(icos_r()[icos_v()[i][0]]);
            glVertex3dv(icos_r()[icos_v()[i][1]]);
            glVertex3dv(icos_r()[icos_v()[i][2]]);
        }

        glEnd();
    }
    inline void glutWireRhombicDodecahedron(void)
    {
        using namespace internal;
        int i;

        for (i = 0; i < 12; i++)
        {
            glBegin(GL_LINE_LOOP);
            glNormal3dv(rdod_n()[i]);
            glVertex3dv(rdod_r()[rdod_v()[i][0]]);
            glVertex3dv(rdod_r()[rdod_v()[i][1]]);
            glVertex3dv(rdod_r()[rdod_v()[i][2]]);
            glVertex3dv(rdod_r()[rdod_v()[i][3]]);
            glEnd();
        }
    }
    inline void glutSolidRhombicDodecahedron(void)
    {
        using namespace internal;
        int i;

        glBegin(GL_QUADS);
        for (i = 0; i < 12; i++)
        {
            glNormal3dv(rdod_n()[i]);
            glVertex3dv(rdod_r()[rdod_v()[i][0]]);
            glVertex3dv(rdod_r()[rdod_v()[i][1]]);
            glVertex3dv(rdod_r()[rdod_v()[i][2]]);
            glVertex3dv(rdod_r()[rdod_v()[i][3]]);
        }

        glEnd();
    }
    inline void glutWireSierpinskiSponge(int num_levels, GLdouble offset[3], GLdouble scale)
    {
        using namespace internal;
        int i, j;

        if (num_levels == 0)
        {

            for (i = 0; i < 4; i++)
            {
                glBegin(GL_LINE_LOOP);
                glNormal3d(-tet_r()[i][0], -tet_r()[i][1], -tet_r()[i][2]);
                for (j = 0; j < 3; j++)
                {
                    double x = offset[0] + scale * tet_r()[tet_i()[i][j]][0];
                    double y = offset[1] + scale * tet_r()[tet_i()[i][j]][1];
                    double z = offset[2] + scale * tet_r()[tet_i()[i][j]][2];
                    glVertex3d(x, y, z);
                }

                glEnd();
            }
        }
        else if (num_levels > 0)
        {
            GLdouble local_offset[3];  /* Use a local variable to avoid buildup of roundoff errors */
            num_levels--;
            scale /= 2.0;
            for (i = 0; i < 4; i++)
            {
                local_offset[0] = offset[0] + scale * tet_r()[i][0];
                local_offset[1] = offset[1] + scale * tet_r()[i][1];
                local_offset[2] = offset[2] + scale * tet_r()[i][2];
                glutWireSierpinskiSponge(num_levels, local_offset, scale);
            }
        }
    }
    inline void glutSolidSierpinskiSponge(int num_levels, GLdouble offset[3], GLdouble scale)
    {
        using namespace internal;
        {

        };
        int i, j;

        if (num_levels == 0)
        {
            glBegin(GL_TRIANGLES);

            for (i = 0; i < 4; i++)
            {
                glNormal3d(-tet_r()[i][0], -tet_r()[i][1], -tet_r()[i][2]);
                for (j = 0; j < 3; j++)
                {
                    double x = offset[0] + scale * tet_r()[tet_i()[i][j]][0];
                    double y = offset[1] + scale * tet_r()[tet_i()[i][j]][1];
                    double z = offset[2] + scale * tet_r()[tet_i()[i][j]][2];
                    glVertex3d(x, y, z);
                }
            }

            glEnd();
        }
        else if (num_levels > 0)
        {
            GLdouble local_offset[3];  /* Use a local variable to avoid buildup of roundoff errors */
            num_levels--;
            scale /= 2.0;
            for (i = 0; i < 4; i++)
            {
                local_offset[0] = offset[0] + scale * tet_r()[i][0];
                local_offset[1] = offset[1] + scale * tet_r()[i][1];
                local_offset[2] = offset[2] + scale * tet_r()[i][2];
                glutSolidSierpinskiSponge(num_levels, local_offset, scale);
            }
        }
    }
}
