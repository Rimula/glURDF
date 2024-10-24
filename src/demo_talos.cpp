#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <learnopengl/shader.h>
#include <learnopengl/camera.h>
#include <learnopengl/model.h>
#include "urdf_parser/urdf_parser.h"
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <stb_image.h>
#include "pprinter.hh"
#include "value-pprint.hh"
#include "tinyusdz.hh"
#include "tydra/render-data.hh"
#include "tydra/scene-access.hh"

using namespace urdf;
using namespace std;

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow *window);
void checkOpenGLError(const std::string& location);
void renderScene(const Shader &shader, bool move);
void renderRobot(const Shader &shader, vector<shared_ptr<Model_>> &meshes, bool move);

// settings
const unsigned int SCR_WIDTH = 1280;
const unsigned int SCR_HEIGHT = 720;
bool replayKeyPressed = false;
bool shadows = true;
bool shadowsKeyPressed = false;
float light_color = 0.6;
int MAX_TICK = 50;
int tick = 0;
int c = 0;

// camera
learnOpenGL::Camera camera(glm::vec3(1.5f, 0.5f, 12.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// file name
const string delimiter1 = "../../resources/talos_data/";
const string delimiter2 = "../../resources/h1_description/";
const string delimiter3 = "package://talos_data/";
const string delimiter4 = "package://h1_description/";
const string urdf_file_path1 = "../../resources/talos_data/urdf/talos_reduced.urdf";
const string urdf_file_path2 = "../../resources/h1_description/urdf/h1_with_hand.urdf";
string config_file_path = "../../resources/configs/";
string scene_file_path = "../../resources/scenes/";

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// bool LoadUSDModel(const std::string& filename, GLuint& VAO, GLsizei& indexCount) {
//     tinyusdz::Stage stage;
//     std::string warn, err;
//     bool success = tinyusdz::LoadUSDAFromFile(filename, &stage, &warn, &err);

//     if (!success) {
//         std::cerr << "Error loading USD: " << err << std::endl;
//         return false;
//     }

//     tinyusdz::tydra::RenderSceneConverterEnv env(stage);
//     tinyusdz::tydra::RenderSceneConverter converter;
//     tinyusdz::tydra::RenderScene renderScene;

//     if (!converter.ConvertToRenderScene(env, &renderScene)) {
//         std::cerr << "Error converting USD stage to RenderScene" << std::endl;
//         return false;
//     }

//     // Use renderScene to initialize OpenGL resources (e.g., VAO, VBO, etc.)
//     glGenVertexArrays(1, &VAO);
//     glBindVertexArray(VAO);

//     GLuint VBO, EBO;
//     glGenBuffers(1, &VBO);
//     glGenBuffers(1, &EBO);

//     // Example usage based on renderScene data (pseudo code)
//     // Assuming renderScene contains relevant vertex data in renderScene.meshes
//     if (renderScene.meshes.empty()) {
//         std::cerr << "No meshes found in USD scene." << std::endl;
//         return false;
//     }

//     const auto& mesh = renderScene.meshes[0];
//     std::vector<float> vertices;
//     for (const auto& point : mesh.points) {
//         vertices.push_back(point[0]);
//         vertices.push_back(point[1]);
//         vertices.push_back(point[2]);
//     }

//     std::vector<unsigned int> indices = mesh.faceVertexIndices();
//     indexCount = static_cast<GLsizei>(indices.size());

//     glBindBuffer(GL_ARRAY_BUFFER, VBO);
//     glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    
//     glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
//     glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

//     // Position attribute
//     glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
//     glEnableVertexAttribArray(0);

//     glBindVertexArray(0);
//     return true;
// }

// void renderUSDModel(Shader& shader, GLuint VAO, GLsizei indexCount) {
//     shader.use();
//     glBindVertexArray(VAO);
//     glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
//     glBindVertexArray(0);
// }

// configurations file
vector<vector<float>> configs;
vector<shared_ptr<Model_>> meshes1;
vector<shared_ptr<Model_>> meshes2;
vector<shared_ptr<Model_>> scenes;
vector<glm::mat4> initTransMat1;
vector<glm::mat4> initTransMat2;
const glm::mat4 init = glm::mat4(1.0f);

glm::mat4 calTransMat (shared_ptr<Model_> link)
{    
    if (link->parent == nullptr)
        return link->joint_transmat;
        
    return calTransMat(link->parent) * link->joint_transmat;
}

void addChildLinks(LinkConstSharedPtr link, shared_ptr<Model_> parent, vector<shared_ptr<Model_>> &meshes, const string &basePath)
{
    double roll, pitch, yaw;
    double x, y, z;
    double roll_, pitch_, yaw_;
    double x_, y_, z_;
    double axis_x, axis_y, axis_z = 0.0f;
    GeometrySharedPtr geom;
    string token;
    string file_name;
    shared_ptr<Model_> child_node;
    
    for (vector<LinkSharedPtr>::const_iterator child = link->child_links.begin(); child != link->child_links.end(); child++)
    {
        (*child)->parent_joint->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw); // rotation
        x = (*child)->parent_joint->parent_to_joint_origin_transform.position.x;
        y = (*child)->parent_joint->parent_to_joint_origin_transform.position.y;
        z = (*child)->parent_joint->parent_to_joint_origin_transform.position.z;

        // UNKNOWN 0  REVOLUTE 1 CONTINUOUS 2 PRISMATIC 3 FLOATING 4 PLANAR 5 FIXED 6
        axis_x = (*child)->parent_joint->axis.x;
        axis_y = (*child)->parent_joint->axis.y;
        axis_z = (*child)->parent_joint->axis.z;
        glm::vec3 axis = glm::vec3(axis_x, axis_y, axis_z);
            
        if ((*child)->visual)
        {
            geom = (*child)->visual->geometry;
            glm::mat4 joint_transmat = init;
            joint_transmat = glm::translate(joint_transmat, glm::vec3(x, y, z)); 
            joint_transmat = glm::rotate(joint_transmat, float(roll), glm::vec3(-1., 0., 0.)); 
            joint_transmat = glm::rotate(joint_transmat, float(pitch), glm::vec3(0., -1., 0.)); 
            joint_transmat = glm::rotate(joint_transmat, float(yaw), glm::vec3(0., 0., -1.));     

            glm::mat4 vis_transmat = init;
            (*child)->visual->origin.rotation.getRPY(roll_, pitch_, yaw_); // rotation
            x_ = (*child)->visual->origin.position.x;
            y_ = (*child)->visual->origin.position.y;
            z_ = (*child)->visual->origin.position.z;
            vis_transmat = glm::translate(vis_transmat, glm::vec3(x_, y_, z_)); 
            vis_transmat = glm::rotate(vis_transmat, float(roll_), glm::vec3(1., 0., 0.)); 
            vis_transmat = glm::rotate(vis_transmat, float(pitch_), glm::vec3(0., 1., 0.)); 
            vis_transmat = glm::rotate(vis_transmat, float(yaw_), glm::vec3(0., 0., 1.));   

            string mat_name;
            
            if (geom->type == MESH)
            {   
                MeshSharedPtr m = urdf::dynamic_pointer_cast<urdf::Mesh>(geom);
                string filename = m->filename;

                // 替换 package:// 前缀为 basePath
                if (filename.find("package://") == 0) {
                    if (filename.find("talos_data") != string::npos) {
                        filename.replace(0, 21, delimiter1);
                    } else if (filename.find("h1_description") != string::npos) {
                        filename.replace(0, 25, delimiter2);
                    }
                 } else if (filename.find("../meshes") == 0) {
               filename.replace(0, 9, basePath + "meshes");
                }

                file_name = filename;

                // 日志信息，帮助调试文件加载问题
                cout << "Attempting to load mesh file: " << file_name << endl;

                // 检查文件是否存在
                ifstream file_check(file_name);
                if (!file_check.good()) {
                    cerr << "ERROR: Mesh file not found: " << file_name << endl;
                    continue;
                }

                // 材质设置
                if ((*child)->visual->material != nullptr)
                    mat_name = (*child)->visual->material->name;
                else
                    mat_name = "White";

                glm::vec3 scale = glm::vec3(m->scale.x, m->scale.y, m->scale.z);
                shared_ptr<Model_> child_node(new Model_((*child)->name, file_name, mat_name, (*child)->parent_joint->type, joint_transmat, vis_transmat, scale, axis, parent));
                if (mat_name == "" && (*child)->visual->material != nullptr)  
                    child_node->color = glm::vec3((*child)->visual->material->color.r, (*child)->visual->material->color.g, (*child)->visual->material->color.b);

                meshes.push_back(child_node);
                addChildLinks(*child, child_node, meshes, basePath);
            }
        }
    }
}


// // 加载OBJ文件的函数
// Model loadModel(const std::string &path) {
//     return Model(path);
// }
std::string getFileExtension(const std::string& fileName) {
    size_t dotPos = fileName.find_last_of(".");
    if (dotPos == std::string::npos) {
        return ""; // No extension found
    }
    return fileName.substr(dotPos);
}


int main(int argc, char** argv)
{
    // if (argc < 3) {
    //     cerr << "USAGE ./demo_talos scene_name [robot_urdf_file_1] [robot_urdf_file_2] ..." << endl;
    //     return -1;
    // }
    if (argc != 2){
        cerr << "USAGE ./demo_talos scene_name" << endl;
        cerr << "SCENE_NAME CHOICES ground stairs bridge" << endl;
        return -1;
    }

    std::string fileName = argv[1];
    std::string extension = getFileExtension(fileName);

    if (extension == ".stl" || extension == ".obj" || extension == ".glb") {
        scene_file_path += fileName; // 使用带扩展名的文件名
    } else {
        cerr << "ERROR: Unsupported file extension. Please use .stl, .obj, or .glb files." << endl;
        return -1;
    }
    config_file_path += fileName.substr(0, fileName.find_last_of(".")); // 去除扩展名，用于配置文件

    // read configs file
    // -------------------------    
    ifstream config_file(config_file_path.data());
    if (config_file.is_open()){
        string line;
        int i = 0;
        int len = 0;
        vector<float> config;
        while (getline(config_file, line)){
            // length of the config in the very first line of the text file
            if (i == 0) len = stof(line);   
            else {
                if (i % len != 0)
                    config.push_back(stof(line));
                else {
                    config.push_back(stof(line));
                    configs.push_back(config);
                    config.clear();
                }
            }
            i++;
        }
        config_file.close();
    }
  
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    
    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "glURDF", NULL, NULL);
    if (window == NULL)
    {
        cout << "Failed to create GLFW window" << endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        cout << "Failed to initialize GLAD" << endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    // build and compile shaders
    // -------------------------
    Shader shader("point_shadows.vs", "point_shadows.fs");
    Shader simpleDepthShader("point_shadows_depth.vs", "point_shadows_depth.fs", "point_shadows_depth.gs");

    float floorVertices[] = {
        // positions          // normals         // texture coords
        50.0f, -1.0f,  50.0f,  0.0f, 1.0f, 0.0f,  1.0f,  0.0f,
        -50.0f, -1.0f,  50.0f,  0.0f, 1.0f, 0.0f,  0.0f,  0.0f,
        -50.0f, -1.0f, -50.0f,  0.0f, 1.0f, 0.0f,  0.0f,  1.0f,

        50.0f, -1.0f,  50.0f,  0.0f, 1.0f, 0.0f,  1.0f,  0.0f,
        -50.0f, -1.0f, -50.0f,  0.0f, 1.0f, 0.0f,  0.0f,  1.0f,
        50.0f, -1.0f, -50.0f,  0.0f, 1.0f, 0.0f,  1.0f,  1.0f
    };

    unsigned int floorVAO, floorVBO;
    glGenVertexArrays(1, &floorVAO);
    glGenBuffers(1, &floorVBO);
    glBindVertexArray(floorVAO);
    glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(floorVertices), floorVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

////xinzneg
    // Load and create texture
    unsigned int textureID;
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Load image, create texture and generate mipmaps
    int width, height, nrChannels;
    unsigned char *data = stbi_load("../../resources/scenes/sea.jpg", &width, &height, &nrChannels, 0);
    if (data)
    {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    }
    else
    {
        std::cout << "Failed to load texture" << std::endl;
    }
    stbi_image_free(data);
////xinzeng
    // configure depth map FBO
    // -----------------------
    const unsigned int SHADOW_WIDTH = 1024, SHADOW_HEIGHT = 1024;
    unsigned int depthMapFBO;
    glGenFramebuffers(1, &depthMapFBO);
    // create depth cubemap texture
    unsigned int depthCubemap;
    glGenTextures(1, &depthCubemap);
    glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
    for (unsigned int i = 0; i < 6; ++i)
    glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    // attach depth texture as FBO's depth buffer
    glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
    glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthCubemap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);


    // shader configuration
    // --------------------
    shader.use();
    shader.setInt("diffuseTexture", 0);
    shader.setInt("depthMap", 1);

    // lighting info
    // -------------
    glm::vec3 lightPos(5.0f, 5.0f, 5.0f);
    
    // draw in wireframe    
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    
     // urdf: read urdf files
    // -------------------------
    string xml_string;
    fstream xml_file1(urdf_file_path1, fstream::in);
    while ( xml_file1.good() )
    {
        string line;
        getline( xml_file1, line);
        xml_string += (line + "\n");
    }
    xml_file1.close();
    ModelInterfaceSharedPtr robot1 = parseURDF(xml_string);
    if (!robot1){
        cerr << "ERROR: Model Parsing the xml failed" << endl;
        return -1;
    }

    xml_string.clear();
    fstream xml_file2(urdf_file_path2, fstream::in);
    while ( xml_file2.good() )
    {
        string line;
        getline( xml_file2, line);
        xml_string += (line + "\n");
    }
    xml_file2.close();
    ModelInterfaceSharedPtr robot2 = parseURDF(xml_string);
    if (!robot2){
        cerr << "ERROR: Model Parsing the xml failed" << endl;
        return -1;
    }

    // load robot models
    // -----------    
    LinkConstSharedPtr link1 = robot1->getRoot();
    GeometrySharedPtr geom1 = link1->visual->geometry;
    MeshSharedPtr m1 = urdf::dynamic_pointer_cast<Mesh>(geom1);
    string token1 = m1->filename.substr(m1->filename.find(delimiter3)+delimiter3.length(), m1->filename.length());
    string file_name1 = delimiter1 + token1;
    glm::vec3 scale1 = glm::vec3(1.0f); glm::vec3 axis1 = glm::vec3(0.0f);
    
    shared_ptr<Model_> root_node1(new Model_ (link1->name, file_name1, link1->visual->material->name, 0, init, init, scale1, axis1, nullptr));
    meshes1.push_back(root_node1);
    //addChildLinks(link1, root_node1, meshes1);
    addChildLinks(link1, root_node1, meshes1, delimiter1);

    LinkConstSharedPtr link2 = robot2->getRoot();
    GeometrySharedPtr geom2 = link2->visual->geometry;
    MeshSharedPtr m2 = urdf::dynamic_pointer_cast<Mesh>(geom2);
    string token2 = m2->filename.substr(m2->filename.find(delimiter4)+delimiter4.length(), m2->filename.length());
    string file_name2 = delimiter2 + token2;
    glm::vec3 scale2 = glm::vec3(1.0f); glm::vec3 axis2 = glm::vec3(0.0f);
    
    shared_ptr<Model_> root_node2(new Model_ (link2->name, file_name2, link2->visual->material->name, 0, init, init, scale2, axis2, nullptr));
    meshes2.push_back(root_node2);
    //addChildLinks(link2, root_node2, meshes2);
    addChildLinks(link2, root_node2, meshes2, delimiter2);


    //init robot pose and scene
    for (int i = 0; i < meshes1.size(); i++)
    {    
        if (i == 0)
        {
            // init pose
            glm::mat4 tmp = glm::rotate(init, glm::radians(90.0f), glm::vec3(-1.0f,0.0f,0.0f));  
            tmp = glm::rotate(tmp, glm::radians(90.0f), glm::vec3(0.0f,0.0f,-1.0f)); 
            tmp = glm::translate(tmp, glm::vec3(0.0f, 0.0f, 0.3f)); 
            meshes1[i]->joint_transmat = tmp;
        }        
        initTransMat1.push_back(meshes1[i]->joint_transmat);
    }    

    for (int i = 0; i < meshes2.size(); i++)
    {    
        if (i == 0)
        {
            // init pose
            glm::mat4 tmp = glm::rotate(init, glm::radians(90.0f), glm::vec3(-1.0f, 0.0f, 0.0f));  
            tmp = glm::rotate(tmp, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, -1.0f));
            tmp = glm::translate(tmp, glm::vec3(5.0f, 2.5f, 0.0f));  // 为第二个机器人加上一些位移（例如向 x 轴方向移动 5 个单位）
            meshes2[i]->joint_transmat = tmp;
        }        
        initTransMat2.push_back(meshes2[i]->joint_transmat);
    }

    // add a scene mesh. can be mutliple.
    // ---------------------------------------
    shared_ptr<Model_> scene(new Model_ ("scene", scene_file_path, "White", 0, init, init, glm::vec3(1.0), glm::vec3(1.0), nullptr));
    glm::mat4 tmp = glm::rotate(init, glm::radians(90.0f), glm::vec3(-1.0f,0.0f,0.0f));  
    tmp = glm::rotate(tmp, glm::radians(90.0f), glm::vec3(0.0f,0.0f,-1.0f));  
    scene->joint_transmat = tmp;
    scenes.push_back(scene);
    
    tick = 0;
    c = 0;
    // render loop
    // -----------
    // Load USD model
    // GLuint usdModelVAO;
    // GLsizei indexCount;
    // std::string usd_file_path = "input.usd";
    // if (argc > 1) {
    //     usd_file_path = argv[1];
    // }
    // if (!LoadUSDModel(usd_file_path, usdModelVAO, indexCount)) {
    //     std::cerr << "Failed to load USD model" << std::endl;
    //     return -1;
    // }

while (!glfwWindowShouldClose(window))
    {
        tick ++; 
        // per-frame time logic
        // --------------------
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.15f, 0.15f, 0.15f, 1.0f);
        //glClearColor(0.5f,0.8f,0.9f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        
        // create depth cubemap transformation matrices
        // -----------------------------------------------
        float near_plane = 1.0f;
        float far_plane = 2500.0f;
        glm::mat4 shadowProj = glm::perspective(glm::radians(90.0f), (float)SHADOW_WIDTH / (float)SHADOW_HEIGHT, near_plane, far_plane);
        std::vector<glm::mat4> shadowTransforms;
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f, 0.0f, 1.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));
        shadowTransforms.push_back(shadowProj * glm::lookAt(lightPos, lightPos + glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, -1.0f, 0.0f)));

        // render scene to depth cubemap
        // --------------------------------
        glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
        glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
        glClear(GL_DEPTH_BUFFER_BIT);
        simpleDepthShader.use();
        for (unsigned int i = 0; i < 6; ++i)
        simpleDepthShader.setMat4("shadowMatrices[" + std::to_string(i) + "]", shadowTransforms[i]);
        simpleDepthShader.setFloat("far_plane", far_plane);
        simpleDepthShader.setVec3("lightPos", lightPos);
        renderScene(simpleDepthShader, true);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        // render scene as normal 
        // -------------------------
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        shader.use();
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        shader.setMat4("projection", projection);
        shader.setMat4("view", view);
        // set lighting uniforms
        shader.setVec3("lightPos", lightPos);
        shader.setVec3("viewPos", camera.Position);
        shader.setInt("shadows", shadows); // enable/disable shadows by pressing 'SPACE'
        shader.setFloat("far_plane", far_plane);
        shader.setVec3("lightColor", glm::vec3(light_color));
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubemap);
        renderScene(shader, true);

        // Render floor  
        shader.use();
        glm::mat4 model = glm::mat4(1.0f);
        shader.setMat4("model", model);
        shader.setVec3("color", glm::vec3(0.6f, 0.6f, 0.6f));
        glActiveTexture(GL_TEXTURE5);
        glBindTexture(GL_TEXTURE_2D, textureID);
        glBindVertexArray(floorVAO);
        glDrawArrays(GL_TRIANGLES, 0, 6);
        glBindVertexArray(0);

        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
        // glDeleteVertexArrays(1, &floorVAO);
        // glDeleteBuffers(1, &floorVBO);
        // glDeleteTextures(1, &textureID);
    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

void renderRobot(const Shader &shader, vector<shared_ptr<Model_>> &meshes, bool move, vector<glm::mat4> &initTransMat)
{
    //glEnable(GL_CULL_FACE);
    int cnt = 7; // HACKING!!
    for (int i = 0; i < meshes.size(); i++)
    {          
        if (tick % MAX_TICK == 0 && c < configs.size())
        {
            if (i == 0)
                meshes[i]->joint_transmat = glm::translate(initTransMat[i], glm::vec3(configs[c][0], configs[c][1], configs[c][2]));  
            if (meshes[i]->axis != glm::vec3(0.0f))
            {
                if (meshes[i]->joint_type == 1) // revolute
                    meshes[i]->joint_transmat = glm::rotate(initTransMat[i], configs[c][cnt], meshes[i]->axis);  
                else if (meshes[i]->joint_type == 3) // prismatic
                    meshes[i]->joint_transmat = glm::translate(initTransMat[i], configs[c][cnt] * meshes[i]->axis);  
                cnt++;
            }
        }
        glm::mat4 transmat = init;
        transmat = glm::scale(transmat, meshes[i]->scale);
        transmat = meshes[i]->vis_transmat * transmat;

        for (int j = 0; j < 3; j++)
        {
            if (meshes[i]->scale[j] < 0)
                glDisable(GL_CULL_FACE);
        }

        if (!meshes[i]->textures_loaded.empty())
            shader.setInt("textures", 1);
        else
            shader.setInt("textures", 0);
        
        transmat = calTransMat(meshes[i]) * transmat;
        shader.setVec3("color", meshes[i]->color);
        shader.setMat4("model", transmat);
        meshes[i]->Draw(shader);
    }
}


void renderScene(const Shader &shader, bool move)
{
    glEnable(GL_CULL_FACE);
    
    for (int i = 0; i < scenes.size(); i ++)
    {
        if (!scenes[i]->textures_loaded.empty())
            shader.setInt("textures", 1);
        else
            shader.setInt("textures", 0);
        shader.setVec3("color", scenes[i]->color);
        shader.setMat4("model", scenes[i]->joint_transmat);
        scenes[i]->Draw(shader);
    }
    // Render both robots
    renderRobot(shader, meshes1, true, initTransMat1);
    renderRobot(shader, meshes2, false, initTransMat2);
    if (tick % MAX_TICK == 0 && move) c++;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // camera pos
    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);

       // Adjust pitch
    if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
        camera.ProcessPitch(1.0f);  // Adjust the value as needed
    if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
        camera.ProcessPitch(-1.0f); // Adjust the value as needed
        
    // // adjust pitch
    // if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    //     camera.ProcessPitch(1.0f);  // Adjust the value as needed
    // if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    //     camera.ProcessPitch(-1.0f); // Adjust the value as needed
    // if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    //     camera.ProcessRoll(1.0f);  // Adjust the value as needed
    // if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    //     camera.ProcessRoll(-1.0f); // Adjust the value as needed
    // light color
    if (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS)
    {
        if (light_color > 0.0)
            light_color -= 0.05;
    }
    if (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS)
    {
        if (light_color < 1.0)
            light_color += 0.05;
    }
    
    // replay the motion
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS && !replayKeyPressed)
    {
        replayKeyPressed = true;
        tick = 0; c= 0;
    }
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_RELEASE)
    {
        replayKeyPressed = false;
    }
    
    // shadow on/off
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS && !shadowsKeyPressed)
    {
        shadows = !shadows;
        shadowsKeyPressed = true;
    }
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_RELEASE)
    {
        shadowsKeyPressed = false;
    }
        if (glfwGetKey(window, GLFW_KEY_O) == GLFW_PRESS) {
        camera.Position = glm::vec3(2.0f, 2.0f, 2.0f); // 重新设置摄像机位置
    }
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(yoffset);
}

void checkOpenGLError(const std::string& location) {
    GLenum err;
    while ((err = glGetError()) != GL_NO_ERROR) {
        std::cerr << "OpenGL error at " << location << ": " << err << std::endl;
    }
}
