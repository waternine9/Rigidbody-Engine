

#include <iostream>
#include "EngineDePhysiques.h"

#include "Renderer.hpp"

#include <fstream>
#include <strstream>
#include <omp.h>
class BaseModel
{
public:
    std::vector<int> indices;
    std::vector<PhysVector3> vertices;
    std::vector<PhysVector2> uvs;
    std::vector<Phys::SubModel> submodels;
    PhysVector3 position;
    void Load(std::string filename, PhysVector3 initPos)
    {
        std::ifstream f(filename);

        position = { 0.0, 0.0, 0.0 };
        int counter = 0, counter2 = 0;
        bool encountered = false;
        Phys::SubModel currentSubmodel;
        currentSubmodel.position = { 0.0, 0.0, 0.0 };
        std::vector<PhysVector2> tempUvs;
        std::vector<PhysVector3> tempVertices;
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
                position = position + v + initPos;
                counter++;
            }
            if (line[0] == 'v' && line[1] == 't')
            {
                PhysVector2 v;
                s >> junk >> junk >> v.x >> v.y;
                tempUvs.push_back(v);
            }
            if (line[0] == 'f')
            {
                int f0, uv0;
                int f1, uv1;
                int f2, uv2;
                s >> junk >> f0 >> junk >> uv0 >> f1 >> junk >> uv1 >> f2 >> junk >> uv2;

                f0--; f1--; f2--;
                uv0--; uv1--; uv2--;
                uvs[f0] = tempUvs[uv0];
                uvs[f1] = tempUvs[uv1];
                uvs[f2] = tempUvs[uv2];
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
        for (PhysVector2 &uv : uvs)
        {
            
            // std::cout << uv.x << " " << uv.y << std::endl;
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
    model->Load("Bruh.obj", { 0.0, -40, 0.0 });
    models.push_back(model);
    model = new BaseModel();
    Phys::Collider* collider = new Phys::Collider();
    collider->Velocity.z = 0.5;
    collider->Init(models[0]->vertices, models[0]->indices, &models[0]->submodels);
    collider->AssignId(0);
    colliders.push_back(collider);

    model = new BaseModel();
    model->Load("map.obj", { 0.0, 0.0, 0.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Static = true;
    collider->Init(model->vertices, model->indices, &model->submodels);
    collider->GenerateOctree();
    colliders.push_back(collider);

    double rotating = 0;
    
    Renderer renderer;
    
    std::cout << "start" << std::endl;
    renderer.CreateWindow("Rigidbodies!", 2560, 1440);
    std::cout << "end" << std::endl;
    renderer.Init();
    std::cout << "Checkpoint0 " << std::endl;
    renderer.AddModel(&models[0]->vertices, models[0]->uvs, models[0]->indices, "texture.jpg");
    renderer.AddModel(&models[1]->vertices, models[1]->uvs, models[1]->indices, "texture.jpg");
    renderer.camera.fov = 90.0f;
    
    while (renderer.LoopUntilClosed())
    {

        const int SUBSTEPS = 64;
        if (renderer.IsKeyPressed(GLFW_KEY_E))
        {
            rotating += 0.00005;
        }
        if (renderer.IsKeyPressed(GLFW_KEY_Q))
        {
            rotating -= 0.00005;
        }
        if (renderer.IsKeyPressed(GLFW_KEY_A))
        {
            colliders[0]->AngularVelocity = colliders[0]->AngularVelocity + inverse(colliders[0]->Rotation) * PhysVector3(0, 0, 0.001);
        }
        if (renderer.IsKeyPressed(GLFW_KEY_D))
        {
            colliders[0]->AngularVelocity = colliders[0]->AngularVelocity - inverse(colliders[0]->Rotation) * PhysVector3(0, 0, 0.001);
        }
        if (renderer.IsKeyPressed(GLFW_KEY_W))
        {
            colliders[0]->AngularVelocity = colliders[0]->AngularVelocity + inverse(colliders[0]->Rotation) * PhysVector3(0.001, 0, 0.0);
        }
        if (renderer.IsKeyPressed(GLFW_KEY_S))
        {
            colliders[0]->AngularVelocity = colliders[0]->AngularVelocity - inverse(colliders[0]->Rotation) * PhysVector3(0.001, 0, 0.0);
        }
        const size_t size = colliders.size();

        float start = clock();
        for (int n = 0; n < SUBSTEPS; n++)
        {
            for (int i = 0; i < size; i++)
            {
                Phys::Collider* collider = colliders[i];
                if (!collider->Static)
                {
                    collider->GenerateOctree();

                }
            }
            for (int i = 0; i < size; i++)
            {
                Phys::Collider* collider = colliders[i];
                if (collider->Static) continue;
                collider->Step(SUBSTEPS, colliders);
            }
            for (int i = 0; i < size; i++)
            {
                Phys::Collider* collider = colliders[i];
                colliders[0]->RotateSubmodels("Propeller", { 0.0, rotating, 0.0 });

                if (renderer.IsKeyPressed(GLFW_KEY_W))
                {
                    colliders[0]->RotateSubmodels("Wheel", { 0.01, 0.0, 0.0 });
                }
                if (renderer.IsKeyPressed(GLFW_KEY_S))
                {
                    colliders[0]->RotateSubmodels("Wheel", { -0.01, 0.0, 0.0 });
                }
                

                collider->Velocity.y += 0.0001;
                collider->Update(SUBSTEPS);
                if (collider->Static) continue;
                PhysVector3 diff = colliders[5]->Position - collider->Position;
                 // scale(collider->Velocity + scale(normalize(diff), 0.01 / magnitude(diff)), 0.999);
            }
            

        }
        PhysVector3 forward = colliders[0]->Rotation * PhysVector3(0.0, 4, 10.0);
        PhysVector3 v = colliders[0]->Position - forward;
        renderer.camera.position.x = v.x;
        renderer.camera.position.y = v.y;
        renderer.camera.position.z = v.z;
        v = colliders[0]->Position;
        renderer.camera.target.x = v.x;
        renderer.camera.target.y = v.y;
        renderer.camera.target.z = v.z;
        renderer.Update();
        renderer.Render();
        
    }
}
