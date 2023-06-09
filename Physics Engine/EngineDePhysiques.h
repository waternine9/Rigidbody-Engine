#pragma once
#include "EngineMath.h"
#include <vector>
#include "Utilities.hpp"

namespace Phys
{
    struct AABB
    {
        PhysVector3 min, max;
    };
    class Triangle
    {
    public:
        PhysVector3 p0, p1, p2, normal;
        AABB bounds;
    };
    
    struct Plane
    {
        PhysVector3 normal;
        PhysVector3 p;
    };



    struct OctreeNode
    {
        OctreeNode* parent = nullptr;
        OctreeNode* branches[8] = { nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr, nullptr };
        bool isLeaf = false;
        PhysVector3 center;

        std::vector<int> indices;
        AABB bounds;
        Plane midPlanes[3];
    };
    bool AABBvsPoint(AABB& a, PhysVector3 &b)
    {

        return (
            a.min.x <= b.x &&
            a.max.x >= b.x &&
            a.min.y <= b.y &&
            a.max.y >= b.y &&
            a.min.z <= b.z &&
            a.max.z >= b.z
            );
    }
    class Octree
    {
    protected:
        bool AABBIntersect(AABB& a, AABB b)
        {

            return (
                a.min.x <= b.max.x &&
                a.max.x >= b.min.x &&
                a.min.y <= b.max.y &&
                a.max.y >= b.min.y &&
                a.min.z <= b.max.z &&
                a.max.z >= b.min.z
                );
        }
        
        bool AABBTriangle(AABB aabb, Triangle triangle)
        {


            AABB triangleBounds = { triangle.p0, triangle.p0 };
            triangleBounds.min = min(triangleBounds.min, triangle.p1);
            triangleBounds.max = max(triangleBounds.max, triangle.p1);
            triangleBounds.min = min(triangleBounds.min, triangle.p2);
            triangleBounds.max = max(triangleBounds.max, triangle.p2);
            return AABBIntersect(aabb, triangleBounds);
        }
    public:
        OctreeNode root;
        int depth = 0;

        ~Octree()
        {
            int stackPointer = 0;
            std::vector<int> stack(depth, 0);
            OctreeNode* current = &root;
            while (true)
            {
                OctreeNode* node = current->branches[stack[stackPointer]];
                if (node)
                {
                    if (node->isLeaf)
                    {
                        delete node;
                        stack[stackPointer]++;
                    }
                    else
                    {
                        stackPointer++;
                        current = node;
                    }
                }
                else
                {
                    stack[stackPointer]++;
                }
                while (stack[stackPointer] > 7)
                {
                    stack[stackPointer] = 0;
                    if (current->parent == nullptr)
                    {
                        return;
                    }
                    current = current->parent;
                    stackPointer--;
                    
                    delete current->branches[stack[stackPointer]];
                    stack[stackPointer]++;
                }
            }
        }
        void Init(std::vector<Triangle> triangles, AABB bounds, int depth)
        {
            int totalTris = 0;
            this->depth = depth;
            root.bounds = bounds;
            return;
            root.center = scale((bounds.min + bounds.max), 0.5f);
            root.midPlanes[0] = { { 1.0f, 0.0f, 0.0f }, root.center };
            root.midPlanes[1] = { { 0.0f, 1.0f, 0.0f }, root.center };
            root.midPlanes[2] = { { 0.0f, 0.0f, 1.0f }, root.center };
            root.indices = std::vector<int>();
            std::vector<AABB> trianglesBounds;

            for (int i = 0; i < triangles.size(); i++)
            {
                root.indices.push_back(i);
            }
            std::vector<int> stack(depth);
            int stackPointer = 0;
            OctreeNode* parent = nullptr, * current = &root;
            while (true)
            {
                AABB currentBounds = {

                        current->bounds.min,
                        current->center
                };
                const double stepX = (current->bounds.max.x - current->bounds.min.x) * 0.5f;
                const double stepY = (current->bounds.max.y - current->bounds.min.y) * 0.5f;
                const double stepZ = (current->bounds.max.z - current->bounds.min.z) * 0.5f;

                switch (stack[stackPointer])
                {
                case 0:
                    break;
                case 1:
                    currentBounds = {
                        currentBounds.min + PhysVector3(stepX, 0.0f, 0.0f),
                        currentBounds.max + PhysVector3(stepX, 0.0f, 0.0f)
                    };
                    break;
                case 2:
                    currentBounds = {
                        currentBounds.min + PhysVector3(0.0f, stepY, 0.0f),
                        currentBounds.max + PhysVector3(0.0f, stepY, 0.0f)
                    };
                    break;
                case 3:
                    currentBounds = {
                        currentBounds.min + PhysVector3(stepX, stepY, 0.0f),
                        currentBounds.max + PhysVector3(stepX, stepY, 0.0f)
                    };
                    break;
                case 4:
                    currentBounds = {
                        currentBounds.min + PhysVector3(0.0f, 0.0f, stepZ),
                        currentBounds.max + PhysVector3(0.0f, 0.0f, stepZ)
                    };
                    break;
                case 5:
                    currentBounds = {
                        currentBounds.min + PhysVector3(stepX, 0.0f, stepZ),
                        currentBounds.max + PhysVector3(stepX, 0.0f, stepZ)
                    };
                    break;
                case 6:
                    currentBounds = {
                        currentBounds.min + PhysVector3(0.0f, stepY, stepZ),
                        currentBounds.max + PhysVector3(0.0f, stepY, stepZ)
                    };
                    break;
                case 7:
                    currentBounds = {
                        currentBounds.min + PhysVector3(stepX, stepY, stepZ),
                        currentBounds.max + PhysVector3(stepX, stepY, stepZ)
                    };
                    break;
                }
                PhysVector3 center = scale((currentBounds.min + currentBounds.max), 0.5f);

                bool isEmpty = true;
                for (int triangle : current->indices)
                {

                    if (AABBTriangle(currentBounds, triangles[triangle]))
                    {
                        isEmpty = false;
                        break;
                    }
                }
                if (!isEmpty)
                {

                    OctreeNode* branch = new OctreeNode;
                    current->branches[stack[stackPointer]] = branch;
                    branch->parent = current;
                    branch->bounds = currentBounds;
                    branch->center = center;

                    if (stackPointer == depth - 1)
                    {
                        branch->isLeaf = true;
                        stack[stackPointer]++;
                    }
                    else
                    {
                        branch->indices = std::vector<int>();
                        for (int triangle : current->indices)
                        {
                            if (AABBTriangle(branch->bounds, triangles[triangle]))
                            {
                                branch->indices.push_back(triangle);
                            }
                        }
                        stackPointer++;
                        parent = current;
                        branch->parent = current;
                        current = branch;
                    }
                }
                else
                {
                    stack[stackPointer]++;
                }
                while (stack[stackPointer] > 7)
                {
                    stack[stackPointer] = 0;
                    if (parent == nullptr)
                    {
                        return;
                    }
                    current = parent;
                    parent = parent->parent;
                    stackPointer--;
                    stack[stackPointer]++;
                }
            }

        }
        
        int GetDeepestNodeAt(PhysVector3 x)
        {
            if (!AABBvsPoint(root.bounds, x)) return 0;
            return depth;
            /*
            OctreeNode* rootNode = &root;
            int n = 0;
            while (rootNode)
            {
                int octant = (rootNode->center.x < x.x) + (rootNode->center.y < x.y) * 2 + (rootNode->center.z < x.z) * 4;
                OctreeNode* next = rootNode->branches[octant];
                if (!next) return n;
                n++;
                rootNode = next;
            }
            return n;*/ 
        }
    };
    bool RayTriangleIntersect(
        PhysVector3& orig, PhysVector3 dir,
        PhysVector3& v0, PhysVector3& v1, PhysVector3& v2,
        double& t)
    {
        PhysVector3 v0v1 = v1 - v0;
        PhysVector3 v0v2 = v2 - v0;
        PhysVector3 pvec = cross(dir, v0v2);
        double det = dot(v0v1, pvec);
        // ray and triangle are parallel if det is close to 0
        if (fabs(det) < 0.00001f) return false;
        double invDet = 1 / det;
        PhysVector3 tvec = orig - v0;
        double u = dot(tvec, pvec) * invDet;
        if (u < 0 || u > 1) return false;
        PhysVector3 qvec = cross(tvec, v0v1);
        double v = dot(dir, qvec) * invDet;
        if (v < 0 || u + v > 1) return false;
        t = dot(v0v2, qvec) * invDet;
        return t > 0.0f;
    }

    bool rayTriangleIntersectWithLimit(PhysVector3 p0, PhysVector3 p1, PhysVector3 p2, PhysVector3 rayPos, PhysVector3 rayVec, double limit, PhysVector3& intersection, double& distance)
    {
        if (RayTriangleIntersect(rayPos, rayVec, p0, p1, p2, distance))
        {
            if (distance <= limit)
            {
                intersection = rayPos + scale(rayVec, distance);
                return true;
            }
        }
        return false;
    }
    bool AABBvsRay(PhysVector3& rayPos, PhysVector3 rayVec, AABB aabb, float& distance)
    {
        if (rayPos.x > aabb.min.x &&
            rayPos.y > aabb.min.y &&
            rayPos.z > aabb.min.z &&
            rayPos.x < aabb.max.x &&
            rayPos.y < aabb.max.y &&
            rayPos.z < aabb.max.z)
        {
            distance = 0;
            return true;
        }
        float tx1 = (aabb.min.x - rayPos.x) / rayVec.x;
        float tx2 = (aabb.max.x - rayPos.x) / rayVec.x;

        float tmin = std::min(tx1, tx2);
        float tmax = std::max(tx1, tx2);

        float ty1 = (aabb.min.y - rayPos.y) / rayVec.y;
        float ty2 = (aabb.max.y - rayPos.y) / rayVec.y;

        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));

        float tz1 = (aabb.min.z - rayPos.z) / rayVec.z;
        float tz2 = (aabb.max.z - rayPos.z) / rayVec.z;

        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));

        distance = tmin;
        return tmax >= tmin && tmax > 0.0f;
    };
    bool AABBvsAABB(AABB& a, AABB b)
    {

        return (
            a.min.x <= b.max.x &&
            a.max.x >= b.min.x &&
            a.min.y <= b.max.y &&
            a.max.y >= b.min.y &&
            a.min.z <= b.max.z &&
            a.max.z >= b.min.z
            );
    }
    

    class Collider;

	struct CollisionInfo
	{
		PhysVector3 Normal;
        PhysVector3 Velocity;
        double Mass;
	};
	struct TriangleProp
	{
        
		PhysVector3* p0;
		PhysVector3* p1;
		PhysVector3* p2;
        PhysVector3 normal;
        double drag;
	};
    class Collider;
    struct Weld
    {
        Collider* other;
        PhysVector3 toOther;
    };
    struct SubModel
    {
        std::string name;
        std::vector<int> indices;
        std::vector<PhysVector3> ownVerts;
        PhysVector3 position;
        PhysQuaternion rotation = { 0, 0, 0, 1 };
    };
	class Collider
	{
	protected:
        std::vector<PhysVector3> _NextVertices;
        std::vector<SubModel> *_SubModels;
        int _Id = 0;

        bool _CollideWithOthers(std::vector<Collider*>& other, PhysVector3 rayOrigin, CollisionInfo &out)
        {
            

            PhysVector3 minNormal(0, 0, 0);
            PhysVector3 minVelocity(0, 0, 0);
            double minMass = 0.0;
            
            int collisions = 0;
            bool isHit = false;
            int n = -1;
            for (Collider* collider : other)
            {
                
                n++;
                


                if (n == _Id) continue;
                
                if (!AABBvsPoint(collider->Bounds, rayOrigin)) continue;
                
                double minSide = -1000000.0;

                bool hitAtAll = true;

                PhysVector3 _minNormal;

                for (TriangleProp& prop : collider->Triangles)
                {



                    double side = dot(prop.normal, *prop.p1 - rayOrigin);
                    
                    if (side > 0)
                    {
                        
                        hitAtAll = false;
                        break;
                    }
                    if (minSide < side)
                    {
                        minSide = side;
                        _minNormal = scale(prop.normal, minSide);
                    }

                    
                }
                if (hitAtAll)
                {
                    collisions++;
                    minNormal = minNormal + _minNormal;
                    minVelocity = minVelocity + collider->Velocity;
                    minMass += collider->Mass;
                    isHit = true;
                }
            }
            out.Normal = scale(minNormal, 1.0 / collisions);
            out.Velocity = scale(minVelocity, 1.0 / collisions);
            out.Mass = minMass / collisions;
            return isHit;
        }
        
	public:
        AABB Bounds;
        PhysVector3 AngularVelocity{ 0.0, 0.0, 0.0 };
        PhysQuaternion Rotation{ 0.0, 0.0, 0.0, 1.0 };
        PhysVector3 Velocity{ 0.0, 0.0, 0.0 }, Position{ 0.0, 0.0, 0.0 }, IterPosition{ 0.0, 0.0, 0.0 };
		std::vector<TriangleProp> Triangles;
        std::vector<int> TriangleIndices;
        std::vector<PhysVector3> RenderVertices;
        std::vector<PhysVector3> _OldVertices;
        std::vector<PhysVector3*> _Vertices;
        bool Static = false;

        PhysVector3 VelocityUpdate;
        PhysVector3 PosUpdate;
        PhysVector3 AngularUpdate;

        int collisions = 0;

        PhysVector3 Forward = { 0.0, 0, 0.2 };

        double Mass = 1;

        void Init(std::vector<PhysVector3>& vertices, std::vector<int>& indices, std::vector<Phys::SubModel>* submodels)
        {
            _Vertices = std::vector<PhysVector3*>();
            _NextVertices = std::vector<PhysVector3>();
            _SubModels = submodels;
            for (PhysVector3& v : vertices)
            {
                _Vertices.push_back(&v);
            }
            Mass = 0.0;
            for (int i = 0; i < indices.size(); i += 3)
            {
                TriangleProp prop;
                prop.p0 = _Vertices[indices[i]];
                prop.p1 = _Vertices[indices[i + 1]];
                prop.p2 = _Vertices[indices[i + 2]];
                prop.normal = normalize(cross(*prop.p2 - *prop.p0, *prop.p1 - *prop.p0));
                Triangles.push_back(prop);
                TriangleIndices.push_back(indices[i]);
                TriangleIndices.push_back(indices[i + 1]);
                TriangleIndices.push_back(indices[i + 2]);
            }
            Position = { 0.0, 0.0, 0.0 };
            for (PhysVector3* _Vert : _Vertices)
            {
                Position = Position + *_Vert;
            }
            Position = scale(Position, 1.0 / _Vertices.size());
            GenerateBounds();
            Mass = (Bounds.max.x - Bounds.min.x) * (Bounds.max.y - Bounds.min.y) * (Bounds.max.z - Bounds.min.z) / 2.0;
            std::cout << Mass << std::endl;
        }
        void GenerateBounds()
        {
            AABB bounds{ { 100000.0, 1000000.0, 1000000.0 }, { -100000.0, -1000000.0, -1000000.0 } };
            for (TriangleProp& prop : Triangles)
            {
                PhysVector3 p0 = *prop.p0;
                PhysVector3 p1 = *prop.p1;
                PhysVector3 p2 = *prop.p2;
                bounds.min = min(bounds.min, p0);
                bounds.max = max(bounds.max, p0);
                bounds.min = min(bounds.min, p1);
                bounds.max = max(bounds.max, p1);
                bounds.min = min(bounds.min, p2);
                bounds.max = max(bounds.max, p2);
            }
            Bounds = bounds;
            Position = { 0.0, 0.0, 0.0 };
            for (PhysVector3* _Vert : _Vertices)
            {
                Position = Position + *_Vert;
            }
            Position = scale(Position, 1.0 / _Vertices.size());

            
            for (TriangleProp &prop : Triangles)
            {
                prop.normal = normalize(cross(*prop.p2 - *prop.p0, *prop.p1 - *prop.p0));
            }
        }
		void Step(int sub, std::vector<Collider*> &otherColliders)
		{
			const double InvSub = 1.0 / sub;
            
            
            Matrix4x4 RotMat = rotate(scale(AngularVelocity, InvSub));

            Forward = RotMat * Forward;

            collisions = 0;
            AngularUpdate = { 0, 0, 0 };
            PosUpdate = { 0, 0, 0 };
            VelocityUpdate = { 0, 0, 0 };


            _OldVertices = std::vector<PhysVector3>();
            for (PhysVector3* v : _Vertices)
            {
                _OldVertices.push_back(*v);
            }

            PhysVector3 toOther;

            Rotation =  ( Rotation + Rotation * (PhysQuaternion{ -AngularVelocity.x * 0.5, -AngularVelocity.y * 0.5, -AngularVelocity.z * 0.5, 0.0 } * InvSub)).normalize();

            IterPosition = IterPosition + scale(Velocity, InvSub);
            UpdateSubmodels();

            
            // AERODYNAMICS
            
            
            for (int prop = 0;prop < TriangleIndices.size();prop += 3)
            {
                PhysVector3 p0 = _OldVertices[TriangleIndices[prop]];
                PhysVector3 p1 = _OldVertices[TriangleIndices[prop + 1]];
                PhysVector3 p2 = _OldVertices[TriangleIndices[prop + 2]];
                PhysVector3 np0 = *_Vertices[TriangleIndices[prop]];
                PhysVector3 np1 = *_Vertices[TriangleIndices[prop + 1]];
                PhysVector3 np2 = *_Vertices[TriangleIndices[prop + 2]];
                double surfaceArea = magnitude(cross(np2 - np0, np1 - np0)) / 2.0;
                PhysVector3 normal = normalize(cross(np2 - np0, np1 - np0));

                double drag = 0.0;
                double tmpDrag = (-dot(normal, np0 - p0) > 0 ? 1 : 0) * magnitude(np0 - p0);
                AngularVelocity = AngularVelocity - inverse(Rotation) * scale(normalize(cross(normal, (Position - np0))), 0.001 / Mass * tmpDrag * surfaceArea * InvSub);
                drag += tmpDrag / 3.0;
                tmpDrag = (-dot(normal, np1 - p1) > 0 ? 1 : 0) * magnitude(np1 - p1);
                AngularVelocity = AngularVelocity - inverse(Rotation) * scale(normalize(cross(normal, (Position - np1))), 0.001 / Mass * tmpDrag * surfaceArea * InvSub);
                drag += tmpDrag / 3.0;
                tmpDrag = (-dot(normal, np2 - p2) > 0 ? 1 : 0) * magnitude(np2 - p2);
                AngularVelocity = AngularVelocity - inverse(Rotation) * scale(normalize(cross(normal, (Position - np2))), 0.001 / Mass * tmpDrag * surfaceArea * InvSub);
                drag += tmpDrag / 3.0;

                drag *= surfaceArea;
                
                drag /= 8000;
                Triangles[prop / 3].drag = drag;
                Velocity = Velocity + scale(normal, drag);
            }
            
            
            Position = { 0.0, 0.0, 0.0 };
            for (PhysVector3* _Vert : _Vertices)
            {
                Position = Position + *_Vert;
            }
            Position = scale(Position, 1.0 / _Vertices.size());

            // AERODYNAMICS

            int _i = -1;
			for (PhysVector3* _Vert : _Vertices)
			{
                _i++;
				PhysVector3 Previous = _OldVertices[_i];
				PhysVector3 Next = *_Vert;
				
				CollisionInfo Result;

				if (_CollideWithOthers(otherColliders, Next, Result))
				{
                    collisions++;
                    PhysVector3 _PosUpdate = scale(Result.Normal, 2.0 * powf(Result.Mass / (Mass + Result.Mass), 1));
					PhysVector3 _VelocityUpdate = scale(scale(Result.Normal, -0.025) + scale(Result.Velocity, 0.01) + scale(Next - Previous, 0.0025), powf(Result.Mass / (Mass + Result.Mass), 1));
                    
                    PosUpdate = PosUpdate + _PosUpdate;

                    VelocityUpdate = VelocityUpdate + _VelocityUpdate;

                    AngularUpdate = AngularUpdate + inverse(Rotation) * scale(cross((Position - Next), Result.Normal), magnitude(Next - Previous) * powf(Result.Mass / (Mass + Result.Mass), 1));
                    
				}
			}

            
        }
        void Update(int sub)
        {
            double InvSub = 1.0 / sub;
            if (collisions > 0)
            {
                Velocity = scale(Velocity - scale(VelocityUpdate, 1.0 / collisions), 1.0 - (0.01 * InvSub));
                AngularVelocity = scale(AngularVelocity + scale(AngularUpdate, 1.0 / collisions), 1.0 - (0.01 * InvSub));
                IterPosition = IterPosition + scale(PosUpdate, 1.0 / collisions);
            }
            RenderVertices = std::vector<PhysVector3>();
            for (PhysVector3* v : _Vertices)
            {
                RenderVertices.push_back(*v);
            }
        }
		void AssignId(int id)
		{
			_Id = id;
		}
        void RotateSubmodels(std::string nameStartsWith, PhysVector3 axis)
        {

            for (SubModel &model : *_SubModels)
            {
                if (startsWith(model.name, nameStartsWith))
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
            for (SubModel &model : *_SubModels)
            {
                for (int i = 0;i < model.indices.size();i += 3)
                {
                    *_Vertices[model.indices[i]] = IterPosition + Rotation * (model.position + model.ownVerts[i]);
                    *_Vertices[model.indices[i + 1]] = IterPosition + Rotation * (model.position + model.ownVerts[i + 1]);
                    *_Vertices[model.indices[i + 2]] = IterPosition + Rotation * (model.position + model.ownVerts[i + 2]); 
                }
            }
        }
	};
}