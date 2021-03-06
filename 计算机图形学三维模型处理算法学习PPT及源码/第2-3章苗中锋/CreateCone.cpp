// CreateCone.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include<cmath>
#define pi 3.1415926
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
int main()
{
	MyMesh mesh;
	int edge,height;
	cout << "edge: " ;
	cin >> edge;
	cout << "height: ";
	cin >> height;
	MyMesh::VertexHandle *vhandle = new MyMesh::VertexHandle[edge + 2];
	for (int i = 0;i < edge;i++) {
		double x = 0.5*cos(pi * 2 / edge * i);
		double y = 0.5*sin(pi * 2 / edge * i);
		vhandle[i] = mesh.add_vertex(MyMesh::Point(x, y, -height/2));
	}

	vhandle[edge] = mesh.add_vertex(MyMesh::Point(0, 0, -height/2));
	vhandle[edge+1] = mesh.add_vertex(MyMesh::Point(0, 0, height/2));
	std::vector<MyMesh::VertexHandle>face_vhandles;
	switch (edge) {
	case 3:
		face_vhandles.clear();
		face_vhandles.push_back(vhandle[0]);
		face_vhandles.push_back(vhandle[2]);
		face_vhandles.push_back(vhandle[1]);
		mesh.add_face(face_vhandles);
		
		face_vhandles.clear();
		face_vhandles.push_back(vhandle[i]);
		face_vhandles.push_back(vhandle[edge + 1]); 
		face_vhandles.push_back(vhandle[next]);
		mesh.add_face(face_vhandles);
		break;
	case 4:
		face_vhandles.clear();
		face_vhandles.push_back(vhandle[0]);
		face_vhandles.push_back(vhandle[2]);
		face_vhandles.push_back(vhandle[1]);
		mesh.add_face(face_vhandles);
		
		face_vhandles.clear();
		face_vhandles.push_back(vhandle[i]);
		face_vhandles.push_back(vhandle[edge + 1]); 
		face_vhandles.push_back(vhandle[next]);
		mesh.add_face(face_vhandles);
		break;
	default:
		for (int i = 0;i < edge;i++) {
			int next = (i + 1) % edge;
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[edge]);
			face_vhandles.push_back(vhandle[i]);
			face_vhandles.push_back(vhandle[next]);
			mesh.add_face(face_vhandles);

			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i]);
			face_vhandles.push_back(vhandle[edge + 1]); 
			face_vhandles.push_back(vhandle[next]);
			mesh.add_face(face_vhandles);
		}
		break;
	}
	
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output3 .off")) {
			std::cerr << "Cannot write mesh to file ' output3 .off ' " << std::endl;
			return 1;
		}
	}
	catch (std::exception&x) {
		std::cerr << x.what() << std::endl;
		return 1;
	}
    return 0;
}

