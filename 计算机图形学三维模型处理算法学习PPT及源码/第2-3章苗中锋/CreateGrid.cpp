// CreateGrid.cpp: 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include<cmath>
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;

int main()
{
	MyMesh mesh;
	int m, n;
	cin >> m >> n;
	double length = 1.0;
	MyMesh::VertexHandle **vhandle = new MyMesh::VertexHandle*[m];
	for (int i = 0;i < m;i++) {
		vhandle[i] = new MyMesh::VertexHandle[n];
	}
	std::vector<MyMesh::VertexHandle>face_vhandles;
	double x = -m* length/2;
	double y = -n* length/2;
	for (int i = 0; i < m; i++) {
		for (int j = 0; j < n; j++) {
			vhandle[i][j] = mesh.add_vertex(MyMesh::Point(x + i * length, y + j * length, 0));
		}
	}
	for (int i = 0; i < m - 1; i++) {
		for (int j = 0; j < n - 1; j++) {

			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i+1][j]);
			face_vhandles.push_back(vhandle[i][j+1]);
			face_vhandles.push_back(vhandle[i][j]);
			
			mesh.add_face(face_vhandles);
			face_vhandles.clear();
			face_vhandles.push_back(vhandle[i + 1][j]);
			face_vhandles.push_back(vhandle[i + 1][j + 1]);
			face_vhandles.push_back(vhandle[i][j + 1]);
			mesh.add_face(face_vhandles);

		}
	}


	// write mesh to output.obj
	try
	{
		if (!OpenMesh::IO::write_mesh(mesh, "output8.off"))
		{
			std::cerr << "Cannot write mesh to file 'output8.off'" << std::endl;
			return 1;
		}
	}
	catch (std::exception& x)
	{
		std::cerr << x.what() << std::endl;
		return 1;
	}

	return 0;
}
