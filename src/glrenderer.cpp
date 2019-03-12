#include <panoramagrid/gl/glrenderer.hpp>
#include <opencv2/core.hpp>

namespace panoramagrid::gl {

    GlRenderer::GlRenderer(int width, int height)
        : Renderer(width, height) {}

    void GlRenderer::render(std::shared_ptr<panoramagrid::Node> node) {
        bindVao(node->getMesh());
        useShader(node->getMaterial());
        bindTextureUnit(node->getMaterial());

        GLint mvpUniform = usedShader->getUniformLocation("mvp");
        glm::mat4 model = glm::translate(toGlm(node->getPosition()));
        auto rpy = getCamera()->getOrientation();
        rpy[2] += M_PI_2f32; // looking along the z axis is 0 yaw
        glm::mat4 view = glm::lookAt(
            glm::vec3(0),
            glm::normalize(glm::vec3(cosf(rpy[2]) * cosf(rpy[1]), sinf(rpy[1]), sinf(rpy[2]) * cosf(rpy[1]))),
            glm::vec3(0, 1, 0)
        );
        glm::mat4 projection = glm::perspective(getCamera()->getFov(), getCamera()->getAspectRatio(), 0.1f, 100.0f);
        glUniformMatrix4fv(mvpUniform, 1, GL_FALSE, glm::value_ptr(projection * view * model));

        glDrawElements(
            getDrawMethod(node->getMesh()->getMethod()),
            static_cast<GLsizei>(node->getMesh()->getIndices().size()),
            GL_UNSIGNED_INT,
            nullptr
        );
    }

    GLenum GlRenderer::getDrawMethod(Mesh::DrawMethod method) {
        return std::map<Mesh::DrawMethod, GLenum>{{Mesh::DrawMethod::TRIANGLE_STRIP, GL_TRIANGLE_STRIP},}.at(method);
    }

    std::map<GLenum, std::pair<int, int>> GlRenderer::getCubemapSides() {
        return std::map<GLenum, std::pair<int, int>>{
            {GL_TEXTURE_CUBE_MAP_POSITIVE_X, std::make_pair<int, int>(2, 1)},
            {GL_TEXTURE_CUBE_MAP_NEGATIVE_X, std::make_pair<int, int>(0, 1)},
            {GL_TEXTURE_CUBE_MAP_POSITIVE_Y, std::make_pair<int, int>(1, 0)},
            {GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, std::make_pair<int, int>(1, 2)},
            {GL_TEXTURE_CUBE_MAP_POSITIVE_Z, std::make_pair<int, int>(1, 1)},
            {GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, std::make_pair<int, int>(3, 1)},
        };
    }

    void GlRenderer::bindVao(std::shared_ptr<Mesh> mesh) {
        GLuint vao;

        try {
            vao = vaos.at(mesh);
            if (vao != boundVao) {
                glBindVertexArray(vao);
            }
        } catch (std::out_of_range &e) {
            GLuint vbo, ebo;
            glGenVertexArrays(1, &vao);
            glGenBuffers(1, &vbo);
            glGenBuffers(1, &ebo);

            glBindVertexArray(vao);
            glBindBuffer(GL_ARRAY_BUFFER, vbo);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);

            glBufferData(
                GL_ARRAY_BUFFER,
                mesh->getVertices().size() * sizeof(GLint),
                mesh->getVertices().data(),
                GL_STATIC_DRAW
            );
            glBufferData(
                GL_ELEMENT_ARRAY_BUFFER,
                mesh->getIndices().size() * sizeof(GLint),
                mesh->getIndices().data(),
                GL_STATIC_DRAW
            );

            glEnableVertexAttribArray(0);
            glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<const void *>(0));

            glEnableVertexAttribArray(1);
            glVertexAttribPointer(
                1,
                2,
                GL_FLOAT,
                GL_FALSE,
                5 * sizeof(float),
                reinterpret_cast<const void *>(3 * sizeof(float))
            );
            glBindVertexArray(vao);

            vaos[mesh] = vao;
        }

        boundVao = vao;
    }

    void GlRenderer::useShader(std::shared_ptr<Material> material) {
        std::shared_ptr<Shader> shader;

        try {
            shader = shaders.at(material);
        } catch (std::out_of_range &e) {
            if (!material->isCubemap()) {
                throw std::invalid_argument("Not implemented");
            }
            shader = std::make_shared<Shader>(R"glsl(
                #version 450 core
                layout (location = 0) in vec3 position;
                layout (location = 1) in vec2 texcoord;
                uniform mat4 mvp;
                uniform bool skybox;
                out vec3 TexCoord;
                void main() {
                    vec4 pos = mvp * vec4(position, 1);
                    gl_Position = pos;
                    if (skybox) {
                        gl_Position = pos.xyww;
                    }
                    TexCoord = position;
                }
                )glsl", R"glsl(
                #version 450 core
                in vec3 TexCoord;
                uniform samplerCube sampler;
                out vec4 color;
                void main() {
                    color = texture(sampler, TexCoord);
                }
                )glsl");
        }

        if (usedShader != shader) {
            shader->use();
            usedShader = shader;
        }
    }

    void GlRenderer::bindTextureUnit(std::shared_ptr<Material> material) {
        if (!material->isCubemap()) {
            throw std::invalid_argument("Not implemented");
        }

        GLenum textureUnit;

        try {
            textureUnit = textureUnits.at(material);
            glActiveTexture(textureUnit);
        } catch (std::out_of_range &e) {
            bool textureUnitsBusy[GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS] = {false};
            for (const auto &texUnitElement : textureUnits) {
                textureUnitsBusy[texUnitElement.second - GL_TEXTURE0] = true;
            }
            int i;
            bool found = false;
            for (i = 0; i < GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS; ++i) {
                if (!textureUnitsBusy[i]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                throw std::runtime_error("There are no free texture units left");
            }
            textureUnit = static_cast<GLenum>(GL_TEXTURE0 + i);

            glActiveTexture(textureUnit);

            GLuint texture;
            glGenTextures(1, &texture);
            glBindTexture(GL_TEXTURE_CUBE_MAP, texture);

            textureUnits[material] = textureUnit;
        }

        glUniform1i(usedShader->getUniformLocation("sampler"), textureUnit - GL_TEXTURE0);
    }

    void GlRenderer::loadTexture(std::shared_ptr<Material> material) {
        if (!material->isCubemap()) {
            throw std::invalid_argument("Not implemented");
        }

        cv::Mat cubemap = material->getTexture();
        int sideDim = cubemap.cols / 4;
        if (sideDim != cubemap.rows / 3) {
            throw std::runtime_error("Invalid cubemap format");
        }

        GLint alignment, rowLength;
        glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
        glGetIntegerv(GL_UNPACK_ROW_LENGTH, &rowLength);

        glPixelStorei(GL_UNPACK_ALIGNMENT, (cubemap.step & 3) ? 1 : 4);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, (GLint) (cubemap.step / cubemap.elemSize()));

        for (auto element : getCubemapSides()) {
            cv::Mat side = cubemap(
                cv::Rect(element.second.first * sideDim, element.second.second * sideDim, sideDim, sideDim));

            glTexImage2D(element.first, 0, GL_RGB, sideDim, sideDim, 0, GL_BGR, GL_UNSIGNED_BYTE, side.ptr());
        }

        glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, rowLength);

        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

        glUniform1i(usedShader->getUniformLocation("skybox"), material->isCubemap());
    }

    glm::vec3 GlRenderer::toGlm(std::array<float, 3> vector) {
        return glm::vec3(vector[0], vector[1], vector[2]);
    }

    cv::Mat GlRenderer::getMat() {
        cv::Mat mat(getHeight(), getWidth(), CV_8UC3);

        glPixelStorei(GL_PACK_ALIGNMENT, (mat.step & 3) ? 1 : 4);
        glPixelStorei(GL_PACK_ROW_LENGTH, static_cast<GLint>(mat.step / mat.elemSize()));
        glReadPixels(0, 0, mat.cols, mat.rows, GL_BGR, GL_UNSIGNED_BYTE, mat.data);
        cv::flip(mat, mat, 0);
        return mat;
    }

}
