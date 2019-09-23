#include <cstring>
#include <panoramagrid/gl/shader.hpp>

namespace panoramagrid::gl {

    Shader::Shader(const std::string &vertexShaderSource, const std::string &fragmentShaderSource) {
        setShaderSource<GL_VERTEX_SHADER>(vertexShaderSource);
        setShaderSource<GL_FRAGMENT_SHADER>(fragmentShaderSource);
    }

    template<GLenum T>
    const std::string Shader::getShaderSource() const {
        throw std::logic_error("Not implemented");
    }

    template<GLenum T>
    void Shader::setShaderSource(const std::string &source) {
        GLuint shader;

        try {
            shader = shaders.at(T);
        } catch (std::out_of_range &e) {
            shader = glCreateShader(T);
            shaders[T] = shader;
        }

        // Upload
        auto *cSource = new GLchar[source.size() + 1];
        strcpy(cSource, source.c_str());
        glShaderSource(shader, 1, &cSource, nullptr);
        delete[] cSource;

        // Compile
        glCompileShader(shader);
        GLint params;
        glGetShaderiv(shader, GL_COMPILE_STATUS, &params);
        if (params != GL_TRUE) {
            glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &params);
            auto infoLog = new GLchar[params];
            glGetShaderInfoLog(shader, params, nullptr, infoLog);
            throw std::logic_error(
                    "The compilation of the " + getShaderName(T) + " was unsuccessful. Info log:\n" + infoLog
            );
        }

        if (program == 0) {
            program = glCreateProgram();
        }

        glAttachShader(program, shader);
        glLinkProgram(program);
    }

    void Shader::use() {
        glUseProgram(program);
    }

    GLint Shader::getUniformLocation(const GLchar *name) {
        GLint uniform = glGetUniformLocation(program, name);
        if (uniform == -1) {
            throw std::logic_error("The name does not correspond to an active uniform variable in the program");
        }
        return uniform;
    }

    std::string Shader::getShaderName(GLenum type) {
        try {
            return std::map<GLenum, std::string>{
                    {GL_VERTEX_SHADER,   "vertex shader"},
                    {GL_FRAGMENT_SHADER, "fragment shader"},
            }.at(type);
        } catch (std::out_of_range &e) {
            return "unknown shader";
        }
    }

    std::string Shader::defaultVertexShader = R"glsl(
        #version 450 core
        layout (location = 0) in vec3 position;
        layout (location = 1) in vec2 texcoord;
        uniform mat4 mvp;
        uniform bool skybox;
        out vec3 TexCoord;
        void main() {
            vec4 pos = mvp * vec4(position, 1);
            if (skybox) {
                gl_Position = pos;
                TexCoord = position;
            } else {
                gl_Position = pos;
                TexCoord = vec3(texcoord, 0);
            }
        }
    )glsl";

    std::string Shader::cubemapFragmentShader = R"glsl(
        #version 450 core
        in vec3 TexCoord;
        uniform samplerCube sampler;
        out vec4 color;
        void main() {
            color = texture(sampler, TexCoord);
        }
    )glsl";

    std::string Shader::sphereFragmentShader = R"glsl(
        #version 450 core
        in vec3 TexCoord;
        uniform sampler2D sampler;
        out vec4 color;
        void main() {
            color = texture(sampler, TexCoord.xy);
        }
    )glsl";
}