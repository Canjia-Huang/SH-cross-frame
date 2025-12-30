#pragma once
#include <GL/glew.h>
#include <boost/optional.hpp>
#include <Eigen/Core>

namespace kt84 {

namespace graphics_util {
    enum struct ParamName {
        #if defined(KT84_USE_OPENGL_4)
        #include "ParamName4.inl"
        #elif defined(KT84_USE_OPENGL_3_3)
        #include "ParamName3.3.inl"
        #else
        #include "ParamName2.1.inl"
        #endif
    };
    inline int    glGet1i(ParamName pname) { int    value; glGetIntegerv(static_cast<GLenum>(pname), &value); return value; }
    inline float  glGet1f(ParamName pname) { float  value; glGetFloatv  (static_cast<GLenum>(pname), &value); return value; }
    inline double glGet1d(ParamName pname) { double value; glGetDoublev (static_cast<GLenum>(pname), &value); return value; }
    template <class EigenType> inline EigenType glGetXi(ParamName pname) { EigenType value; glGetIntegerv(static_cast<GLenum>(pname), value.data()); return value; }
    template <class EigenType> inline EigenType glGetXf(ParamName pname) { EigenType value; glGetFloatv  (static_cast<GLenum>(pname), value.data()); return value; }
    template <class EigenType> inline EigenType glGetXd(ParamName pname) { EigenType value; glGetDoublev (static_cast<GLenum>(pname), value.data()); return value; }
    
    inline Eigen::Matrix4d get_modelview_matrix() {
        return glGetXd<Eigen::Matrix4d>(ParamName::MODELVIEW_MATRIX);
    }
    inline Eigen::Matrix4d get_projection_matrix() {
        return glGetXd<Eigen::Matrix4d>(ParamName::PROJECTION_MATRIX);
    }
    inline Eigen::Vector4i get_viewport() {
        return glGetXi<Eigen::Vector4i>(ParamName::VIEWPORT);
    }
    inline float read_depth(int x, int y) {
        float z = 0;
    	glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &z);
        return z;
    }
    inline Eigen::Vector3d unproject(const Eigen::Vector3d& win_xyz,
                                     const Eigen::Matrix4d& modelview_matrix,
                                     const Eigen::Matrix4d& projection_matrix,
                                     const Eigen::Vector4i& viewport)
    {
        Eigen::Vector3d obj_xyz;
    	gluUnProject( win_xyz.x(),  win_xyz.y(),  win_xyz.z(),     modelview_matrix.data(), projection_matrix.data(), viewport.data(),
                     &obj_xyz.x(), &obj_xyz.y(), &obj_xyz.z());
        return obj_xyz;
    }
    inline Eigen::Vector3d unproject(const Eigen::Vector3d& win_xyz) {
        return unproject(win_xyz, get_modelview_matrix(), get_projection_matrix(), get_viewport());
    }
    inline boost::optional<Eigen::Vector3d> read_and_unproject(int x, int y) {
        auto modelview_matrix  = get_modelview_matrix();
        auto projection_matrix = get_projection_matrix();
        auto viewport          = get_viewport();
        
        float z = read_depth(x, y);
        if (z == 1.0f)
            return boost::none;
        
        return unproject(Eigen::Vector3d(x, y, z));
    }
    inline Eigen::Vector3d project(const Eigen::Vector3d& obj_xyz,
                                   const Eigen::Matrix4d& modelview_matrix,
                                   const Eigen::Matrix4d& projection_matrix,
                                   const Eigen::Vector4i& viewport)
    {
        Eigen::Vector3d win_xyz;
    	gluProject( obj_xyz.x(),  obj_xyz.y(),  obj_xyz.z(),    modelview_matrix.data(), projection_matrix.data(), viewport.data(),
                   &win_xyz.x(), &win_xyz.y(), &win_xyz.z());
        return win_xyz;
    }
    inline Eigen::Vector3d project(const Eigen::Vector3d& obj_xyz) {
        return project(obj_xyz, get_modelview_matrix(), get_projection_matrix(), get_viewport());
    }
    // interface to GL functions with arbitrary data type
    // glColor
#define KT84_DEFINE_GLCOLOR(suffix) \
    template<typename T> void glColor3##suffix(T&& vec) { ::glColor3##suffix(vec[0], vec[1], vec[2]); } \
    template<typename T> void glColor4##suffix(T&& vec) { ::glColor4##suffix(vec[0], vec[1], vec[2], vec[3]); } \
    template<typename TRGB, typename TAlpha> void glColor4##suffix(TRGB&& rgb, TAlpha&& alpha) { ::glColor4##suffix(rgb[0], rgb[1], rgb[2], alpha); }
#define KT84_DEFINE_GLCOLOR_I(suffix) KT84_DEFINE_GLCOLOR(suffix) KT84_DEFINE_GLCOLOR(u##suffix)
    KT84_DEFINE_GLCOLOR(f)
    KT84_DEFINE_GLCOLOR(d)
    KT84_DEFINE_GLCOLOR_I(i)
    KT84_DEFINE_GLCOLOR_I(b)
    KT84_DEFINE_GLCOLOR_I(s)
#undef KT84_DEFINE_GLCOLOR_I
#undef KT84_DEFINE_GLCOLOR
    // glVertex
#define KT84_DEFINE_GLVERTEX(suffix) \
    template<typename T> void glVertex2##suffix(T&& vec) { ::glVertex2##suffix(vec[0], vec[1]); } \
    template<typename T> void glVertex3##suffix(T&& vec) { ::glVertex3##suffix(vec[0], vec[1], vec[2]); } \
    template<typename T> void glVertex4##suffix(T&& vec) { ::glVertex4##suffix(vec[0], vec[1], vec[2], vec[3]); }
    KT84_DEFINE_GLVERTEX(f)
    KT84_DEFINE_GLVERTEX(d)
    KT84_DEFINE_GLVERTEX(i)
    KT84_DEFINE_GLVERTEX(s)
#undef KT84_DEFINE_GLVERTEX_I
#undef KT84_DEFINE_GLVERTEX
    // glNormal
#define KT84_DEFINE_GLNORMAL(suffix) \
    template<typename T> void glNormal3##suffix(T&& vec) { ::glNormal3##suffix(vec[0], vec[1], vec[2]); }
    KT84_DEFINE_GLNORMAL(f)
    KT84_DEFINE_GLNORMAL(d)
    KT84_DEFINE_GLNORMAL(i)
    KT84_DEFINE_GLNORMAL(b)
    KT84_DEFINE_GLNORMAL(s)
#undef KT84_DEFINE_GLNORMAL
    // glTexCoord
#define KT84_DEFINE_GLTEXCOORD(suffix) \
    template<typename T> void glTexCoord2##suffix(T&& vec) { ::glTexCoord2##suffix(vec[0], vec[1]); } \
    template<typename T> void glTexCoord3##suffix(T&& vec) { ::glTexCoord3##suffix(vec[0], vec[1], vec[2]); } \
    template<typename T> void glTexCoord4##suffix(T&& vec) { ::glTexCoord4##suffix(vec[0], vec[1], vec[2], vec[3]); }
    KT84_DEFINE_GLTEXCOORD(f)
    KT84_DEFINE_GLTEXCOORD(d)
    KT84_DEFINE_GLTEXCOORD(i)
    KT84_DEFINE_GLTEXCOORD(s)
#undef KT84_DEFINE_GLTEXCOORD
    // glTranslate
    template <typename T> void glTranslated(T&& vec) { ::glTranslated(vec[0], vec[1], vec[2]); }
    template <typename T> void glTranslatef(T&& vec) { ::glTranslatef(vec[0], vec[1], vec[2]); }
    // glScale
    template <typename T> void glScaled(T&& vec) { ::glScaled(vec[0], vec[1], vec[2]); }
    template <typename T> void glScalef(T&& vec) { ::glScalef(vec[0], vec[1], vec[2]); }
    inline void glScaled(double s) { ::glScaled(s, s, s); }
    inline void glScalef(float  s) { ::glScalef(s, s, s); }
    // glRotate
    template <typename T> void glRotated(T&& angleaxis) { ::glRotated(angleaxis[0], angleaxis[1], angleaxis[2], angleaxis[3]); }
    template <typename T> void glRotatef(T&& angleaxis) { ::glRotatef(angleaxis[0], angleaxis[1], angleaxis[2], angleaxis[3]); }
    template <typename T> void glRotated(double angle, T&& axis) { ::glRotated(angle, axis[0], axis[1], axis[2]); }
    template <typename T> void glRotatef(float  angle, T&& axis) { ::glRotatef(angle, axis[0], axis[1], axis[2]); }
    // gluLookAt
    template <typename T1, typename T2, typename T3>
    void gluLookAt(T1&& eye, T2&& center, T3&& up) { ::gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2], up[0], up[1], up[2]); }
    // glRasterPos
#define KT84_DEFINE_GLRASTERPOS(suffix) \
    template<typename T> void glRasterPos2##suffix(T&& vec) { ::glRasterPos2##suffix(vec[0], vec[1]); } \
    template<typename T> void glRasterPos3##suffix(T&& vec) { ::glRasterPos3##suffix(vec[0], vec[1], vec[2]); } \
    template<typename T> void glRasterPos4##suffix(T&& vec) { ::glRasterPos4##suffix(vec[0], vec[1], vec[2], vec[3]); }
    KT84_DEFINE_GLRASTERPOS(s)
    KT84_DEFINE_GLRASTERPOS(i)
    KT84_DEFINE_GLRASTERPOS(f)
    KT84_DEFINE_GLRASTERPOS(d)
#undef KT84_DEFINE_GLRASTERPOS
    // glClearColor
    template <typename T> void glClearColor(T&& vec) { ::glClearColor(vec[0], vec[1], vec[2], vec[3]); }
    template<typename TRGB, typename TAlpha> void glClearColor(TRGB&& rgb, TAlpha&& alpha) { ::glClearColor(rgb[0], rgb[1], rgb[2], alpha); }
}

}
