#pragma once
#include <fstream>
#include <vector>
#include <string>
#include "../ret_msg.hh"

namespace kt84 {
    namespace internal {
        inline void writeMQO_header(std::ostream& out) {
            out << "Metasequoia Document\n";
            out << "Format Text Ver 1.0\n";
        }
        template <class Mesh>
        inline void writeMQO_object(std::ostream& out, const Mesh& mesh) {
            out << "Object {\n";
            out << "  vertex " << mesh.n_vertices() << " {\n";
            for (auto v : mesh.vertices()) {
                out << "    ";
                auto& p = mesh.point(v);
                for (int i = 0; i < 3; ++i)
                    out << std::fixed << p[i] << ' ';
                out << '\n';
            }
            out << "  }\n";
            out << "  face " << mesh.n_faces() << " {\n";
            for (auto f : mesh.faces()) {
                out << "    " << mesh.valence(f) << " V(";
                for (auto fv : mesh.fv_range(f))
                    out << fv.idx() << " ";
                out << ")\n";
            }
            out << "  }\n";
            out << "}\n";
        }
        inline void writeMQO_footer(std::ostream& out) {
            out << "Eof\n";
        }
    }
    template <class Mesh>
    inline bool writeMQO(const Mesh& mesh, const std::string& filename) {
        std::ofstream fout(filename.c_str());
        if (!fout.is_open()) return ret_msg<bool>(false, "writeMQO: failed to open file");
        internal::writeMQO_header(fout);
        internal::writeMQO_object(fout, mesh);
        internal::writeMQO_footer(fout);
        return true;
    }
    template <class Mesh>
    inline bool writeMQO(const std::vector<Mesh>& meshes, const std::string& filename) {
        std::ofstream fout(filename.c_str());
        if (!fout.is_open()) return ret_msg<bool>(false, "writeMQO: failed to open file");
        internal::writeMQO_header(fout);
        for (auto& mesh : meshes)
            internal::writeMQO_object(fout, mesh);
        internal::writeMQO_footer(fout);
        return true;
    }
}
