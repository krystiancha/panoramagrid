#ifndef PANORAMAGRID_SHADER_HPP
#define PANORAMAGRID_SHADER_HPP


#include <string>
#include <map>
#include <glad/glad.h>

namespace panoramagrid::gl {

    class Shader {
    public:
        Shader(const std::string &vertexShaderSource, const std::string &fragmentShaderSource);

        template<GLenum T>
        const std::string getShaderSource() const;

        template<GLenum T>
        void setShaderSource(const std::string &source);

        void use();

        GLint getUniformLocation(const GLchar *name);

        static std::string getShaderName(GLenum type);

    private:
        GLuint program = 0;
        std::map<GLenum, GLuint> shaders;
    };

}


#endif //PANORAMAGRID_SHADER_HPP
