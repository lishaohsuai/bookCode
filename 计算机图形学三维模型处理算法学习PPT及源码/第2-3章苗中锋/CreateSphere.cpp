// CreateSphere.cpp: 定义控制台应用程序的入口点。
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
	int n,count=10000,k=-1;
	cout << "n:";
	cin >> n;
	MyMesh::VertexHandle *vhandle = new MyMesh::VertexHandle[count];
	std::vector<MyMesh::VertexHandle>face_vhandles;
	for (int j = -n / 2;j < n / 2;j++) {
		double distance = sin(j*pi / n);
		double r_circle = cos(j*pi / n);
		for (int i = 0;i < n;i++) {
			++k;
			vhandle[k]=mesh.add_vertex(MyMesh::Point(r_circle*cos(2 * i*pi / n), r_circle*sin(2 * i*pi / n), distance));
		
		}
	}
	for (int i = 0;i < n-1;i++) {
		for (int j = 0;j < n;j++) {
			int topRight = i * n + j;
			int topLeft = i * n + (j + 1) % n;
			int bottomRight = (i + 1)*n + j;
			int bottomLeft = (i + 1)*n + (j + 1) % n;
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[topRight]);
			face_vhandles.push_back(vhandle[bottomLeft]);
			face_vhandles.push_back(vhandle[bottomRight]);
			mesh.add_face(face_vhandles);

			face_vhandles.clear();
			face_vhandles.push_back(vhandle[topRight]);
			face_vhandles.push_back(vhandle[topLeft]);
			face_vhandles.push_back(vhandle[bottomLeft]);
			mesh.add_face(face_vhandles);

		}
	}
	vhandle[k+1] = mesh.add_vertex(MyMesh::Point(0, 0, 1));
	vhandle[k+2] = mesh.add_vertex(MyMesh::Point(0, 0, -1));
	for (int i = 0;i < n;i++) {
		face_vhandles.clear();
		face_vhandles.push_back(vhandle[k+2]);
		face_vhandles.push_back(vhandle[(i + 1) % n]);
		face_vhandles.push_back(vhandle[i]);
		mesh.add_face(face_vhandles);

		face_vhandles.clear();
		face_vhandles.push_back(vhandle[k+1]);
		face_vhandles.push_back(vhandle[k- (i + 1) % n]);
		face_vhandles.push_back(vhandle[k- i]);
		mesh.add_face(face_vhandles);
	}
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output5 .off")) {
			std::cerr << "Cannot write mesh to file ' output5 .off ' " << std::endl;
			return 1;
		}
	}
	catch (std::exception&x) {
		std::cerr << x.what() << std::endl;
		return 1;
	}
    return 0;
}

