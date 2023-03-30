

#include <iostream>
#include "EngineDePhysiques.h"

#include "Renderer.hpp"

#include <fstream>
#include <strstream>
#include <omp.h>

int colliderIdx = 0;

class BaseModel
{
public:
    
    std::vector<int> colliderIndices;
    std::vector<PhysVector3> colliderVertices;

    std::vector<int> indices;
    std::vector<PhysVector3> vertices;
    std::vector<PhysVector3> renderVertices;
    std::vector<PhysVector3> normals;
    std::vector<PhysVector2> uvs;
    std::vector<Phys::SubModel> colliderSubmodels;
    std::vector<Phys::SubModel> submodels;
    PhysVector3 position;
    Phys::Collider *collider;
    void LoadCollider(std::string filename, PhysVector3 initPos, bool isStatic)
    {
        std::ifstream f(filename);

        position = { 0.0, 0.0, 0.0 };
        int counter = 0, counter2 = 0;
        bool encountered = false;
        Phys::SubModel currentSubmodel;
        currentSubmodel.position = { 0.0, 0.0, 0.0 };
        std::vector<PhysVector2> tempUvs;
        std::vector<PhysVector3> tempNormals;
        while (!f.eof())
        {
            char line[128];
            f.getline(line, 128);

            std::strstream s;
            s << line;

            char junk;

            if (line[0] == 'o')
            {

                if (encountered)
                {
                    colliderSubmodels.push_back(currentSubmodel);
                    std::string name;
                    currentSubmodel = Phys::SubModel();
                    s >> junk >> name;
                    currentSubmodel.name = name;

                    
                }
                else
                {
                    std::string name;
                    s >> junk >> name;
                    currentSubmodel.name = name;

                    encountered = true;
                }
            }
            if (line[0] == 'v' && line[1] == ' ')
            {
                PhysVector3 v;
                s >> junk >> v.x >> v.y >> v.z;
                colliderVertices.push_back(v + initPos);
                counter++;
            }
            if (line[0] == 'f')
            {
                int f0;
                int f1;
                int f2;
                s >> junk >> f0 >> f1 >> f2;

                f0--; f1--; f2--;
                colliderIndices.push_back(f0);
                colliderIndices.push_back(f1);
                colliderIndices.push_back(f2);
                currentSubmodel.indices.push_back(f0);
                currentSubmodel.indices.push_back(f1);
                currentSubmodel.indices.push_back(f2);
                currentSubmodel.ownVerts.push_back(colliderVertices[f0] - initPos);
                currentSubmodel.ownVerts.push_back(colliderVertices[f1] - initPos);
                currentSubmodel.ownVerts.push_back(colliderVertices[f2] - initPos);
                currentSubmodel.position = currentSubmodel.position + (colliderVertices[f0] - initPos) + (colliderVertices[f1] - initPos) + (colliderVertices[f2] - initPos);
            }
        }
        colliderSubmodels.push_back(currentSubmodel);
        for (Phys::SubModel &model : colliderSubmodels)
        {
            model.position = scale(model.position, 1.0 / model.ownVerts.size());
            for (PhysVector3 &v : model.ownVerts)
            {
                v = v - model.position;
            }
        }
        collider = new Phys::Collider();
        collider->AssignId(colliderIdx);
        collider->Static = isStatic;
        collider->Update(1);
        colliderIdx++;
        collider->Init(colliderVertices, colliderIndices, &colliderSubmodels);
    }
    void Load(std::string filename, PhysVector3 initPos)
    {
        std::ifstream f(filename);

        position = { 0.0, 0.0, 0.0 };
        int counter = 0, counter2 = 0;
        bool encountered = false;
        Phys::SubModel currentSubmodel;
        currentSubmodel.position = { 0.0, 0.0, 0.0 };
        std::vector<PhysVector2> tempUvs;
        std::vector<PhysVector3> tempNormals;
        while (!f.eof())
        {
            char line[128];
            f.getline(line, 128);

            std::strstream s;
            s << line;

            char junk;

            if (line[0] == 'o')
            {

                if (encountered)
                {
                    submodels.push_back(currentSubmodel);
                    std::string name;
                    currentSubmodel = Phys::SubModel();
                    s >> junk >> name;
                    currentSubmodel.name = name;

                    
                }
                else
                {
                    std::string name;
                    s >> junk >> name;
                    currentSubmodel.name = name;

                    encountered = true;
                }
            }
            if (line[0] == 'v' && line[1] == ' ')
            {
                PhysVector3 v;
                s >> junk >> v.x >> v.y >> v.z;
                vertices.push_back(v + initPos);
                uvs.push_back({ 0, 0 });
                normals.push_back({ 0, 0, 0});
                position = position + v + initPos;
                counter++;
            }
            if (line[0] == 'v' && line[1] == 't')
            {
                PhysVector2 v;
                s >> junk >> junk >> v.x >> v.y;
                tempUvs.push_back(v);
            }
            if (line[0] == 'v' && line[1] == 'n')
            {
                PhysVector3 v;
                s >> junk >> junk >> v.x >> v.y >> v.z;
                tempNormals.push_back(v);
            }
            if (line[0] == 'f')
            {
                int f0, uv0, n0;
                int f1, uv1, n1;
                int f2, uv2, n2;
                s >> junk >> f0 >> junk >> uv0 >> junk >> n0 >> f1 >> junk >> uv1 >> junk >> n1 >> f2 >> junk >> uv2 >> junk >> n2;

                f0--; f1--; f2--;
                uv0--; uv1--; uv2--;
                n0--; n1--; n2--;
                uvs[f0] = tempUvs[uv0];
                uvs[f1] = tempUvs[uv1];
                uvs[f2] = tempUvs[uv2];
                normals[f0] = tempNormals[n0];
                normals[f1] = tempNormals[n1];
                normals[f2] = tempNormals[n2];
                indices.push_back(f0);
                indices.push_back(f1);
                indices.push_back(f2);
                currentSubmodel.indices.push_back(f0);
                currentSubmodel.indices.push_back(f1);
                currentSubmodel.indices.push_back(f2);
                currentSubmodel.ownVerts.push_back(vertices[f0] - initPos);
                currentSubmodel.ownVerts.push_back(vertices[f1] - initPos);
                currentSubmodel.ownVerts.push_back(vertices[f2] - initPos);
                currentSubmodel.position = currentSubmodel.position + (vertices[f0] - initPos) + (vertices[f1] - initPos) + (vertices[f2] - initPos);
            }
        }
        submodels.push_back(currentSubmodel);
        position = scale(position, 1.0 / counter);
        for (Phys::SubModel &model : submodels)
        {
            model.position = scale(model.position, 1.0 / model.ownVerts.size());
            for (PhysVector3 &v : model.ownVerts)
            {
                v = v - model.position;
            }
        }
        
    }
    void RotateSubmodels(std::string _startsWith, PhysVector3 axis)
    {
        if (collider)
        {
            collider->RotateSubmodels(_startsWith, axis);
        }
        for (Phys::SubModel &model : submodels)
        {
            if (startsWith(model.name, _startsWith))
            {
                model.rotation = (model.rotation + model.rotation * PhysQuaternion{ -axis.x * 0.5, -axis.y * 0.5, -axis.z * 0.5, 0.0 }).normalize();
                Matrix4x4 rotMat = rotate(axis);

                for (PhysVector3 &v : model.ownVerts)
                {
                    v = rotMat * v;
                }
                for (int i = 0;i < model.indices.size();i += 3)
                {
                    
                    model.ownVerts[i] = rotMat * model.ownVerts[i];
                    model.ownVerts[i + 1] = rotMat * model.ownVerts[i + 1];
                    model.ownVerts[i + 2] = rotMat * model.ownVerts[i + 2];
                }
            }
        }
    }
    void UpdateSubmodels()
    {
        for (Phys::SubModel &model : submodels)
        {
            #pragma omp parallel for
            for (int i = 0;i < model.indices.size();i += 3)
            {
                vertices[model.indices[i]] = collider->IterPosition + collider->Rotation * (model.position + model.ownVerts[i]);
                vertices[model.indices[i + 1]] = collider->IterPosition + collider->Rotation * (model.position + model.ownVerts[i + 1]);
                vertices[model.indices[i + 2]] = collider->IterPosition + collider->Rotation * (model.position + model.ownVerts[i + 2]); 
            }
        }
    }
};
int main()
{
    omp_set_num_threads(8);


    double tick = 0;

    std::vector<BaseModel*> models;
    std::vector<Phys::Collider*> colliders;

    BaseModel* model = new BaseModel();
    model->Load("Bruh.obj", { 0.0, 0, 0.0 });
    std::cout << "Bruh\n";
    model->LoadCollider("testing.obj", { 0.0, 0, 0.0 }, false);
    models.push_back(model);
    std::cout << "Bruh\n";
    model = new BaseModel();
    model->Load("map.obj", { 0.0, 0.0, 0.0 });
    model->LoadCollider("map2.obj", { 0.0, 0.0, 0.0 }, true);
    models.push_back(model);

    for (BaseModel *model : models)
    {
        colliders.push_back(model->collider);
    }

    double rotating = 0;
    
    Renderer renderer;
    
    std::cout << "start" << std::endl;
    renderer.CreateWindow("Rigidbodies!", 2560, 1440);
    std::cout << "end" << std::endl;
    renderer.Init();
    std::cout << "Checkpoint0 " << std::endl;
    renderer.AddModel(&models[0]->vertices, models[0]->uvs, models[0]->normals, models[0]->indices, &models[0]->collider->Rotation, "texture.jpg");
    renderer.AddModel(&models[1]->vertices, models[1]->uvs, models[1]->normals, models[1]->indices, &models[1]->collider->Rotation, "texture2.jpg");
    renderer.camera.fov = 90.0f;
    
    

    while (renderer.LoopUntilClosed())
    {
        
        


        const int SUBSTEPS = 64;
        if (renderer.IsKeyPressed(GLFW_KEY_Q))
        {
            rotating -= 0.005;
        }
        if (renderer.IsKeyPressed(GLFW_KEY_E))
        {
            rotating += 0.005;
        }

        if (renderer.IsKeyPressed(GLFW_KEY_S))
        {
            models[0]->RotateSubmodels("Wing", { 0.001, 0.0, 0.0 });
        }

        if (renderer.IsKeyPressed(GLFW_KEY_W))
        {
            models[0]->RotateSubmodels("Wing", { -0.001, 0.0, 0.0 });
        }

        const size_t size = models.size();

        float start = clock();
        for (int n = 0; n < SUBSTEPS; n++)
        {
            for (int i = 0; i < size; i++)
            {
                BaseModel* model = models[i];
                if (!model->collider->Static)
                {
                    model->collider->GenerateOctree();

                }
            }
            for (int i = 0; i < size; i++)
            {
                BaseModel* model = models[i];
                if (model->collider->Static) continue;
                model->collider->Step(SUBSTEPS, colliders);
            }

            
            for (int i = 0; i < size; i++)
            {
                BaseModel* model = models[i];
                
                
                if (model->collider->Static) continue;
                model->collider->Velocity.y += 0.001 / SUBSTEPS;
                model->collider->Update(SUBSTEPS);
            }
            models[0]->RotateSubmodels("Propeller", { 0.0, rotating / SUBSTEPS, 0.0 });

        }
        models[0]->UpdateSubmodels();
        //models[1]->UpdateSubmodels();
        PhysVector3 forward = models[0]->collider->Rotation * PhysVector3(0.0, 4, 10);
        PhysVector3 v = models[0]->collider->Position - forward;
        renderer.camera.position.x = v.x;
        renderer.camera.position.y = v.y;
        renderer.camera.position.z = v.z;
        v = models[0]->collider->Position;
        renderer.camera.target.x = v.x;
        renderer.camera.target.y = v.y;
        renderer.camera.target.z = v.z;
        renderer.Update();
        renderer.Render();
        
    }
}
