

#include <iostream>
#include "EngineDePhysiques.h"

#include "raylib.h"
#include <fstream>
#include <strstream>
#include <omp.h>
#include <Windows.h>

class BaseModel
{
public:
    std::vector<int> indices;
    std::vector<PhysVector3> vertices;
    PhysVector3 position;
    void Load(std::string filename, PhysVector3 initPos)
    {
        std::ifstream f(filename);

        position = { 0.0, 0.0, 0.0 };
        int counter = 0;
        while (!f.eof())
        {
            char line[128];
            f.getline(line, 128);

            std::strstream s;
            s << line;

            char junk;

            if (line[0] == 'v' && line[1] == ' ')
            {
                PhysVector3 v;
                s >> junk >> v.x >> v.y >> v.z;
                vertices.push_back(v + initPos);
                position = position + v + initPos;
                counter++;
            }
            if (line[0] == 'f')
            {
                int f0;
                int f1;
                int f2;
                s >> junk >> f0 >> f1 >> f2;

                f0--; f1--; f2--;
                indices.push_back(f0);
                indices.push_back(f1);
                indices.push_back(f2);
            }
        }
        position = scale(position, 1.0 / counter);
    }
    void Render(std::vector<PhysVector3>& v)
    {
        for (int i = 0; i < indices.size(); i += 3)
        {

            PhysVector3 p0 = vertices[indices[i]];
            PhysVector3 p1 = vertices[indices[i + 1]];
            PhysVector3 p2 = vertices[indices[i + 2]];
            PhysVector3 normal = normalize(cross(p2 - p0, p1 - p0));
            Vector3 rp0 = { p0.x, p0.y, p0.z };
            Vector3 rp1 = { p1.x, p1.y, p1.z };
            Vector3 rp2 = { p2.x, p2.y, p2.z };
            unsigned char lums = normal.y * 127.0 + 128.0;
            DrawTriangle3D(rp0, rp1, rp2, Color{ lums, lums, lums, 255 });
        }
    }
};

int main()
{
    omp_set_num_threads(8);

    InitWindow(2560, 1440, "Softbodies!");

    SetTargetFPS(60);

    Camera3D camera = { 0 };
    camera.position = { 0.0f, 0.0f, -4.0f };  // Camera position
    camera.target = { 0.0f, 0.0f, 4.0f };      // Camera looking at point
    camera.up = { 0.0f, -1.0f, 0.0f };          // Camera up vector (rotation towards target)
    camera.fovy = 90.0f;                                // Camera field-of-view Y
    camera.projection = CAMERA_PERSPECTIVE;

    double tick = 0;

    std::vector<BaseModel*> models;
    std::vector<Phys::Collider*> colliders;

    BaseModel* model = new BaseModel();
    model->Load("testing.obj", { 0.0, -4, 0.0 });
    models.push_back(model);
    model = new BaseModel();
    Phys::Collider* collider = new Phys::Collider();
    collider->Velocity.z = 0.9;
    collider->Init(models[0]->vertices, models[0]->indices);
    collider->AssignId(0);
    colliders.push_back(collider);

    model = new BaseModel();
    model->Load("wheel.obj", { -1.0, -3.7, -1.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Init(models[1]->vertices, models[1]->indices);
    collider->AssignId(1);
    colliders.push_back(collider);
    //colliders[1]->WeldTo(colliders[0]);

    model = new BaseModel();
    model->Load("wheel.obj", { 1.0, -3.7, -1.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Init(models[2]->vertices, models[2]->indices);
    collider->AssignId(2);
    colliders.push_back(collider);
    //colliders[2]->WeldTo(colliders[0]);

    model = new BaseModel();
    model->Load("wheel.obj", { -1.0, -3.7, 1.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Init(models[3]->vertices, models[3]->indices);
    collider->AssignId(3);
    colliders.push_back(collider);
    //colliders[3]->WeldTo(colliders[0]);

    model = new BaseModel();
    model->Load("wheel.obj", { 1.0, -3.7, 1.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Init(models[4]->vertices, models[4]->indices);
    collider->AssignId(4);
    colliders.push_back(collider);
    //colliders[4]->WeldTo(colliders[0]);

    model = new BaseModel();
    model->Load("map.obj", { 0.0, 0.0, 0.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Static = true;
    collider->Init(model->vertices, model->indices);
    collider->GenerateOctree();

    colliders.push_back(collider);

    model = new BaseModel();
    model->Load("map2.obj", { 0.0, 0.0, 0.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Static = true;
    collider->Init(model->vertices, model->indices);
    collider->GenerateOctree();

    colliders.push_back(collider);

    model = new BaseModel();
    model->Load("map3.obj", { 0.0, 0.0, 0.0 });
    models.push_back(model);
    collider = new Phys::Collider();
    collider->Static = true;
    collider->Init(model->vertices, model->indices);
    collider->GenerateOctree();

    colliders.push_back(collider);

    int key = 0;

    while (!WindowShouldClose())
    {
        const size_t size = colliders.size();


        double mouseX = GetMouseX() / 100.0 - 5.0;
        double mouseY = (GetMouseY() / 30.0 - 5.0) - 2.0;

        const int SUBSTEPS = 4;
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

                collider->Update(SUBSTEPS);
                if (collider->Static) continue;
                PhysVector3 diff = colliders[5]->Position - collider->Position;
                collider->Velocity.y += 0.0004; // scale(collider->Velocity + scale(normalize(diff), 0.01 / magnitude(diff)), 0.999);
            }
            for (int i = 0; i < size; i++)
            {
                Phys::Collider* collider = colliders[i];
                collider->FixWelds(SUBSTEPS);
            }
        }

        PhysVector3 v = colliders[0]->Position - scale(colliders[0]->Forward, 20);
        //camera.position = { (float)v.x, (float)v.y - 2.0f, (float)v.z };
        v = colliders[0]->Position;
        //camera.target = { (float)v.x, (float)v.y, (float)v.z };

        int newKey = GetKeyPressed();
        key = newKey;
        if (key == KEY_N)
        {
            BaseModel* model = new BaseModel();
            model->Load("projectile.obj", colliders[0]->Position + scale(colliders[0]->Forward, 16) + PhysVector3(0, -0.0, 0));
            models.push_back(model);
            Phys::Collider* collider = new Phys::Collider();
            collider->Velocity = scale(colliders[0]->Forward, 4);
            collider->Init(model->vertices, model->indices);
            collider->AssignId(colliders.size());
            collider->Update(SUBSTEPS);
            colliders.push_back(collider);
            key = 0;
        }

        if (GetKeyState('A') & 0x8000)
        {
            colliders[0]->AngularVelocity.y = 0.2; // colliders[1]->AngularVelocity + scale(colliders[0]->Forward, 2);
        }

        if (GetKeyState('D') & 0x8000)
        {
            colliders[0]->AngularVelocity.y = -0.2;
        }
        PhysVector3 rotateBy = colliders[0]->Rotation * PhysVector3{ 0.2, 0, 0.0 };

        if (GetKeyState('W') & 0x8000)
        {
            colliders[1]->AngularVelocity = rotateBy;
            colliders[2]->AngularVelocity = rotateBy;
            colliders[3]->AngularVelocity = rotateBy;
            colliders[4]->AngularVelocity = rotateBy;
        }
        if (GetKeyState('S') & 0x8000)
        {

            colliders[1]->AngularVelocity = scale(rotateBy, -1);
            colliders[2]->AngularVelocity = scale(rotateBy, -1);
            colliders[3]->AngularVelocity = scale(rotateBy, -1);
            colliders[4]->AngularVelocity = scale(rotateBy, -1);
        }
        if (key == KEY_E)
        {
            colliders[0]->Velocity = colliders[0]->Velocity + PhysVector3(0.0f, 0.1f, 0.0f);
        }
        if (key == KEY_Q)
        {
            colliders[0]->Velocity = colliders[0]->Velocity + PhysVector3(0.0f, -0.1f, 0.0f);
        }


        ClearBackground(BLACK);


        BeginDrawing();
        BeginMode3D(camera);

        int t = -1;
        for (BaseModel* model : models)
        {
            t++;
            model->Render(colliders[t]->RenderVertices);
        }

        EndMode3D();
        EndDrawing();

        DrawFPS(10, 10);
    }
}
