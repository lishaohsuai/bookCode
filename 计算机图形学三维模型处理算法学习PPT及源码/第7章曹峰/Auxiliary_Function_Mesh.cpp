#include "InteractiveViewerWidget.h"
#include <Eigen\Dense>
#include <iostream>
#include <cmath>
#include <stack>

void InteractiveViewerWidget::inverse_mesh_connectivity()
{
	Mesh temp_mesh;

	for(Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		temp_mesh.add_vertex(mesh.point(v_it));
	}
	Mesh::FaceIter f_it;
	Mesh::FaceVertexIter fv_it;
	std::vector<Mesh::VertexHandle> face_vertex;
	for(f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
	{
		fv_it = mesh.fv_iter(f_it);
		face_vertex.clear();
		for(fv_it;fv_it;++fv_it)
		{
			face_vertex.push_back(fv_it);
		}
		std::reverse(face_vertex.begin(), face_vertex.end());

		temp_mesh.add_face(face_vertex);
	}

	clearAllMesh();
	temp_mesh.request_vertex_status();
	temp_mesh.request_edge_status();
	temp_mesh.request_face_status();

	temp_mesh.request_face_normals();
	temp_mesh.request_vertex_normals();
	mesh = temp_mesh;
	initMesh();
	setDrawMode(FLAT_POINTS);
	setMouseMode(TRANS);
}

void InteractiveViewerWidget::scale_mesh_using_BBox(int max_len)
{
	double x = bbMax[0] - bbMin[0];
	double y = bbMax[1] - bbMin[1];
	double z = bbMax[2] - bbMin[2];
	double x_dst = max_len; double y_dst = max_len; double z_dst = max_len;
	if(x > y && x > z && x > 0 )
	{
		y_dst = y*x_dst/x;
		z_dst = z*x_dst/x;
	}
	else if(y > z && y > z && y > 0 )
	{
		x_dst = x*y_dst/y;
		z_dst = z*y_dst/y;
	}
	else if(z > x && z > y && z > 0 )
	{
		x_dst = x*z_dst/z;
		y_dst = y*z_dst/z;
	}


	OpenMesh::Vec3d v_pos ;
	for(Mesh::VertexIter vIt = mesh.vertices_begin(); vIt != mesh.vertices_end(); ++vIt)
	{
		v_pos = mesh.point(vIt);
		if(std::abs(x) > 1e-20)
		{
			v_pos[0] = bbMin[0]+(v_pos[0] - bbMin[0])*x_dst/x;
		}
		if(std::abs(y) > 1e-20)
		{
			v_pos[1] = bbMin[1]+(v_pos[1] - bbMin[1])*y_dst/y;
		}
		if(std::abs(z) > 1e-20)
		{
			v_pos[2] = bbMin[2]+(v_pos[2] - bbMin[2])*z_dst/z;
		}
		mesh.set_point(vIt, v_pos);
	}

	first_init = true;
	initMesh();
	setDrawMode(FLAT_POINTS);
	setMouseMode(TRANS);
}

void InteractiveViewerWidget::split_quad_mesh()
{
	if( meshMode() == QUAD )
	{
		Mesh temp_mesh;

		for(Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			temp_mesh.add_vertex(mesh.point(v_it));
		}
		Mesh::FaceIter f_it; Mesh::FaceVertexIter fv_it;
		std::vector<Mesh::VertexHandle> face_vertex;
		std::vector<Mesh::VertexHandle> face0;
		std::vector<Mesh::VertexHandle> face1;
		for(f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it)
		{
			fv_it = mesh.fv_iter(f_it);
			face_vertex.clear();
			for(fv_it;fv_it;++fv_it)
			{
				face_vertex.push_back(fv_it);
			}
			face0.clear(); face0.push_back(face_vertex[0]);  face0.push_back(face_vertex[1]);  face0.push_back(face_vertex[2]);
			face1.clear(); face1.push_back(face_vertex[2]);  face1.push_back(face_vertex[3]);  face1.push_back(face_vertex[0]);
			temp_mesh.add_face(face0); temp_mesh.add_face(face1);
		}

		clearAllMesh();
		temp_mesh.request_vertex_status();
		temp_mesh.request_edge_status();
		temp_mesh.request_face_status();

		temp_mesh.request_face_normals();
		temp_mesh.request_vertex_normals();
		mesh = temp_mesh;
		initMesh();
		setDrawMode(FLAT_POINTS);
		setMouseMode(TRANS);
	}
}

void InteractiveViewerWidget::transform_mesh(const std::vector<double>& m)
{
	if(mesh.n_vertices() ==0) return;

	Eigen::Matrix4d T; Eigen::Vector4d p; Eigen::Vector4d x; OpenMesh::Vec3d vp;
	T(0,0)  = m[0]; T(0,1)  = m[1]; T(0,2)  = m[2]; T(0,3)  = m[3];
	T(1,0)  = m[4]; T(1,1)  = m[5]; T(1,2)  = m[6]; T(1,3)  = m[7];
	T(2,0)  = m[8]; T(2,1)  = m[9]; T(2,2) = m[10]; T(2,3) = m[11];
	T(3,0) = m[12]; T(3,1) = m[13]; T(3,2) = m[14]; T(3,3) = m[15];
	std::cout << T << "\n";
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		vp = mesh.point(v_it.handle());
		p(0) = vp[0]; p(1) = vp[1]; p(2) = vp[2]; p(3) = 1.0;
		x = T * p; 
		if(std::abs( x(3) ) < 1e-10 ) x(3) = 1.0;
		mesh.set_point(v_it.handle(), OpenMesh::Vec3d( x(0)/x(3), x(1)/x(3), x(2)/x(3) ) );
	}
	first_init = true;
	initMesh();
	setDrawMode(FLAT_POINTS);
	setMouseMode(TRANS);
}

void InteractiveViewerWidget::find_vertex_by_id(int id)
{
	if(id < mesh.n_vertices() && id >= 0 )
	{
		selectedVertex.push_back(id);
		updateGL();
	}
}

void InteractiveViewerWidget::find_vertex_by_valance(int valance)
{
	if( valance > 0 )
	{
		selectedVertex.clear();
		for(Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
		{
			if(mesh.valence(v_it) == valance)
			{
				selectedVertex.push_back(v_it.handle().idx() );
			}
		}
		printf("Vertex : %d; %f\n", selectedVertex.size(), double(selectedVertex.size())/mesh.n_vertices() );
		updateGL();
	}
}

void InteractiveViewerWidget::find_face_by_id(int id)
{
	if(id < mesh.n_faces() && id >= 0 )
	{
		selectedFace.push_back(id);
		updateGL();
	}
}

void InteractiveViewerWidget::find_edge_by_id(int id)
{
	if(id < mesh.n_edges() && id >= 0 )
	{
		selectedEdge.push_back(id);
		updateGL();
	}
}

void InteractiveViewerWidget::delete_vertex_valence_four()
{
}

void InteractiveViewerWidget::delete_vertex_valence_three()
{
}

void InteractiveViewerWidget::split_vertex_valence_eight()
{
}

bool EdgeCanCollapse(Mesh mesh, Mesh::HalfedgeHandle h)
{
	Mesh::VertexHandle v1, v2, v3, v4;
	v1 = mesh.to_vertex_handle(h);
	v2 = mesh.from_vertex_handle(h);
	/*v3 = mesh.to_vertex_handle(mesh.next_halfedge_handle(h));
	v4 = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(h)));*/
	if (mesh.is_boundary(v1) || mesh.is_boundary(v2))
	{
		return false;
	}	
	//int right = mesh.valence(v1);//价
	//int left = mesh.valence(v2);
	//int top = mesh.valence(v3);
	//int buttom = mesh.valence(v4);
	//return left + right < 13 && top>5 && buttom > 5;	
}

//最小边长简化
void InteractiveViewerWidget::change_bianchang()
{
	
	int i = 0, j;
	Mesh::HalfedgeHandle h0, h1;
	Mesh::EdgeHandle eh;
	for (Mesh::EdgeIter it = mesh.edges_sbegin(); it != mesh.edges_end(); ++it)
	{
		h0 = mesh.halfedge_handle(it.handle(), 0);
		if (!EdgeCanCollapse(mesh, h0))
		{
			continue;
		}
		if (!mesh.is_boundary(it.handle()))
		{
			bianchang[i].len = mesh.calc_edge_length(it.handle());
			bianchang[i].id = it.handle().idx();
			i++;
		}
	}
	double minlen = bianchang[0].len;
	int minid = bianchang[0].id;
	for (j = 0; j < i; j++)//找到最短的边（内部的）
	{
		if (minlen > bianchang[j].len)
		{
			minlen = bianchang[j].len;
			minid = bianchang[j].id;
		}
	}
	std::cout << "minlen:" << minlen << " minid:" << minid << std::endl;

#pragma region 找到相邻的点
	Mesh::HalfedgeHandle start;//找到相邻的点,有重复
	start = mesh.halfedge_handle(mesh.edge_handle(minid), 0);
	Mesh::VertexHandle from,to;
	from = mesh.from_vertex_handle(start);//from,to最短边的两个点
	int fid = from.idx();
	to = mesh.to_vertex_handle(start);
	int tid = to.idx();
	std::cout << "fid:" << fid << " tid:" << tid << std::endl;
	double zxx, zxy, zxz;
	zxx = (mesh.point(from).data()[0] + mesh.point(to).data()[0]) / 2.0;
	zxy = (mesh.point(from).data()[1] + mesh.point(to).data()[1]) / 2.0;
	zxz = (mesh.point(from).data()[2] + mesh.point(to).data()[2]) / 2.0;

	Mesh::VertexIter v_it;
	int count = 0;//count是点的总数
	for (v_it = mesh.vertices_sbegin(); fid > 0; fid--, v_it++) {}
	for (auto vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
	{
		xianglinpoint[count].x = mesh.point(*vv_it).data()[0];
		xianglinpoint[count].y = mesh.point(*vv_it).data()[1];
		xianglinpoint[count].z = mesh.point(*vv_it).data()[2];
		xianglinpoint[count].id = vv_it.handle().idx();
		count++;
	}

	for (v_it = mesh.vertices_sbegin(); tid > 0; tid--, v_it++) {}
	for (auto vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
	{
		xianglinpoint[count].x = mesh.point(*vv_it).data()[0];
		xianglinpoint[count].y = mesh.point(*vv_it).data()[1];
		xianglinpoint[count].z = mesh.point(*vv_it).data()[2];
		xianglinpoint[count].id = vv_it.handle().idx();
		count++;
	}
	/*for (j = 0; j < count; j++)
		std::cout << xianglinpoint[j].id << "  " << xianglinpoint[j].x << "  " <<
		xianglinpoint[j].y << "  " << xianglinpoint[j].z << std::endl;*///over
#pragma endregion

#pragma region 去除重复点
	int k, l;//去除重复点
	for (k = 0; k < count; k++)
	{
		if (xianglinpoint[k].id == from.idx())
		{
			for (l = k; l < count; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			count--;
		}
	}
	for (k = 0; k < count; k++)
	{
		if (xianglinpoint[k].id == to.idx())
		{
			for (l = k; l < count; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			count--;
		}
	}
	for (k = 0; k < count; k++)
	{
		for (l = k + 1; l < count; l++)
		{
			if (xianglinpoint[k].id == xianglinpoint[l].id)
			{
				for (int m = l; m < count; m++)
				{
					xianglinpoint[m].x = xianglinpoint[m + 1].x;
					xianglinpoint[m].y = xianglinpoint[m + 1].y;
					xianglinpoint[m].z = xianglinpoint[m + 1].z;
					xianglinpoint[m].id = xianglinpoint[m + 1].id;
				}
				count--;
			}
		}
	}
	for (j = 0; j < count; j++)
		std::cout << xianglinpoint[j].id << "  ";
	std::cout<< std::endl;//over
#pragma endregion

#pragma region 杂乱无序的点进行排序
	double x = 0, y = 0;//没有顺序的点进行排序
	for (j = 0; j < count; j++)
	{
		x += xianglinpoint[j].x;
		y += xianglinpoint[j].y;
	}
	center.x = x / count;
	center.y = y / count;
	for (j = 0; j < count - 1; j++)
	{
		for (k = 0; k < count - j - 1; k++)
		{
			bool change;
			if (xianglinpoint[k].x >= 0 && xianglinpoint[k + 1].x < 0)
				change = true;
			else if (xianglinpoint[k].x == 0 && xianglinpoint[k + 1].x == 0)
				change = (xianglinpoint[k].y > xianglinpoint[k + 1].y);
			else
			{
				int det = (xianglinpoint[k].x - center.x) * (xianglinpoint[k + 1].y - center.y) - (xianglinpoint[k + 1].x - center.x) * (xianglinpoint[k].y - center.y);

				if (det < 0)
					change = true;
				else if (det > 0)
					change = false;
				else
				{
					double d1 = (xianglinpoint[k].x - center.x) * (xianglinpoint[k].x - center.x) + (xianglinpoint[k].y - center.y) * (xianglinpoint[k].y - center.y);
					double d2 = (xianglinpoint[k + 1].x - center.x) * (xianglinpoint[k + 1].x - center.y) + (xianglinpoint[k + 1].y - center.y) * (xianglinpoint[k + 1].y - center.y);
					change = (d1 > d2);
				}
			}

			if (change)
			{
				double tmp = xianglinpoint[k].x;
				xianglinpoint[k].x = xianglinpoint[k + 1].x;
				xianglinpoint[k + 1].x = tmp;

				tmp = xianglinpoint[k].y;
				xianglinpoint[k].y = xianglinpoint[k + 1].y;
				xianglinpoint[k + 1].y = tmp;

				tmp = xianglinpoint[k].z;
				xianglinpoint[k].z = xianglinpoint[k + 1].z;
				xianglinpoint[k + 1].z = tmp;

				int tmp1 = xianglinpoint[k].id;
				xianglinpoint[k].id = xianglinpoint[k + 1].id;
				xianglinpoint[k + 1].id = tmp1;
			}
		}
	}
	for (j = 0; j < count; j++)
		std::cout << xianglinpoint[j].id << "  ";
	std::cout << std::endl;//over
#pragma endregion

#pragma region 删除
	mesh.request_face_status();//两个点from，to
	mesh.request_edge_status();
	mesh.request_vertex_status();

	int id1, id2;
	Mesh::VertexIter v_it1, v_it2;
	id1 = from.idx();
	id2 = to.idx();
	for (v_it1 = mesh.vertices_sbegin(); id1 > 0; id1--, v_it1++) {}
	for (v_it2 = mesh.vertices_sbegin(); id2 > 0; id2--, v_it2++) {}

	mesh.delete_vertex(v_it1);
	mesh.delete_vertex(v_it2);
	mesh.garbage_collection();//over
#pragma endregion

#pragma region 连接
	Mesh::VertexHandle zhongxin;//获取中点
	zhongxin = mesh.add_vertex(Mesh::Point(zxx, zxy, zxz));
	Mesh::VertexIter v_lj1, v_lj2;
	Mesh::VertexHandle zj1, zj2;
	for (j = 0; j < count - 1; j++)
	{
		bool v_1 = false, v_2 = false;
		zhuanhuan[0] = xianglinpoint[j];
		zhuanhuan[1] = xianglinpoint[j + 1];
		std::cout << zhuanhuan[0].x << " " << zhuanhuan[0].y << " " << zhuanhuan[0].z << std::endl;
		std::cout << zhuanhuan[1].x << " " << zhuanhuan[1].y << " " << zhuanhuan[1].z << std::endl;
		for (v_lj1 = mesh.vertices_begin(); v_lj1 != mesh.vertices_end(); v_lj1++)
		{
			if (mesh.point(v_lj1).data()[0] == zhuanhuan[0].x&&mesh.point(v_lj1).data()[1] == zhuanhuan[0].y&&mesh.point(v_lj1).data()[2] == zhuanhuan[0].z)
			{
				v_1 = true;
				break;
			}
		}
		for (v_lj2 = mesh.vertices_begin(); v_lj2 != mesh.vertices_end(); v_lj2++)
		{
			if (mesh.point(v_lj2).data()[0] == zhuanhuan[1].x&&mesh.point(v_lj2).data()[1] == zhuanhuan[1].y&&mesh.point(v_lj2).data()[2] == zhuanhuan[1].z)
			{
				v_2 = true;
				break;
			}
		}
		if (!v_1)
			zj1 = mesh.add_vertex(Mesh::Point(zhuanhuan[0].x, zhuanhuan[0].y, zhuanhuan[0].z));
		if (!v_2)
			zj2 = mesh.add_vertex(Mesh::Point(zhuanhuan[1].x, zhuanhuan[1].y, zhuanhuan[1].z));
		if (v_1&&v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;
			mesh.add_face(v_lj1, v_lj2, zhongxin);
			//mesh.garbage_collection();
		}
		else if (!v_1&&v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;
			mesh.add_face(zj1, v_lj2, zhongxin);
			//mesh.garbage_collection();
		}
		else if (v_1 && !v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;
			mesh.add_face(v_lj1, zj2, zhongxin);
			//mesh.garbage_collection();
		}
		else if (!v_1 && !v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;;
			mesh.add_face(zj1, zj2, zhongxin);
			//mesh.garbage_collection();
		}
	}


	bool v_1 = false, v_2 = false;
	zhuanhuan[0] = xianglinpoint[count - 1];
	zhuanhuan[1] = xianglinpoint[0];
	for (v_lj1 = mesh.vertices_begin(); v_lj1 != mesh.vertices_end(); v_lj1++)
	{
		if (mesh.point(v_lj1).data()[0] == zhuanhuan[0].x&&mesh.point(v_lj1).data()[1] == zhuanhuan[0].y&&mesh.point(v_lj1).data()[2] == zhuanhuan[0].z)
		{
			v_1 = true;
			break;
		}
	}
	for (v_lj2 = mesh.vertices_begin(); v_lj2 != mesh.vertices_end(); v_lj2++)
	{
		if (mesh.point(v_lj2).data()[0] == zhuanhuan[1].x&&mesh.point(v_lj2).data()[1] == zhuanhuan[1].y&&mesh.point(v_lj2).data()[2] == zhuanhuan[1].z)
		{
			v_2 = true;
			break;
		}
	}
	if (!v_1)
		zj1 = mesh.add_vertex(Mesh::Point(zhuanhuan[0].x, zhuanhuan[0].y, zhuanhuan[0].z));
	if (!v_2)
		zj2 = mesh.add_vertex(Mesh::Point(zhuanhuan[1].x, zhuanhuan[1].y, zhuanhuan[1].z));
	if (v_1&&v_2)
	{
		std::cout << v_1 << " " << v_2 << std::endl;
		mesh.add_face(v_lj1, v_lj2, zhongxin);
		//mesh.garbage_collection();
	}
	else if (!v_1&&v_2)
	{
		std::cout << v_1 << " " << v_2 << std::endl;
		mesh.add_face(zj1, v_lj2, zhongxin);
		//mesh.garbage_collection();
	}
	else if (!v_1 && !v_2)
	{

		std::cout << v_1 << " " << v_2 << std::endl;;
		mesh.add_face(zj1, zj2, zhongxin);
		//mesh.garbage_collection();
	}
	mesh.garbage_collection();
#pragma endregion
	
	updateGL();
}

//最小面积简化
void InteractiveViewerWidget::change_Area()//面积简化
{

	int xianglinface = 0;
	int i = 0, j, k, l, m;
	double a, b, c, p, area;
	int num = 0;
	for (auto it = mesh.faces_sbegin(); it != mesh.faces_end() ; it++)//找最小面积以及他的id号
	{
		if (!mesh.is_boundary(it))
		{
			m = 0;
			for (auto fv_it = mesh.fv_iter(*it); fv_it.is_valid(); ++fv_it)
			{
				mianji[m].x = mesh.point(*fv_it).data()[0];
				mianji[m].y = mesh.point(*fv_it).data()[1];
				mianji[m].z = mesh.point(*fv_it).data()[2];
				m++;
			}
			a = sqrt((mianji[0].x - mianji[1].x)*(mianji[0].x - mianji[1].x) + (mianji[0].y - mianji[1].y)*(mianji[0].y - mianji[1].y) + (mianji[0].z - mianji[1].z)*(mianji[0].z - mianji[1].z));
			b = sqrt((mianji[0].x - mianji[2].x)*(mianji[0].x - mianji[2].x) + (mianji[0].y - mianji[2].y)*(mianji[0].y - mianji[2].y) + (mianji[0].z - mianji[2].z)*(mianji[0].z - mianji[2].z));
			c = sqrt((mianji[1].x - mianji[2].x)*(mianji[1].x - mianji[2].x) + (mianji[1].y - mianji[2].y)*(mianji[1].y - mianji[2].y) + (mianji[1].z - mianji[2].z)*(mianji[1].z - mianji[2].z));
			p = (a + b + c) / 2.0;
			area = sqrt(p*(p - a)*(p - b)*(p - c));;
			facearea[num].area = area;
			facearea[num].id = it.handle().idx();
			num++;
		}
		//std::cout << "面积：" << facearea[num].area << " id:" << facearea[num].id << std::endl;
	}
	std::cout << num << std::endl;
	double min = facearea[0].area;
	int minid = facearea[0].id;
	/*std::cout << "facearea[0].area：" << facearea[0].area << std::endl;
	std::cout << "facearea[0].id：" << facearea[0].id << std::endl;*/
	for (m = 0; m < num; m++)
	{
		if (min > facearea[m].area)
		{
			min = facearea[m].area;
			minid = facearea[m].id;
		}
	}
	std::cout << "最小面积：" << min << std::endl;
	std::cout << "最小面积id：" << minid << std::endl;
	Mesh::FaceIter f_it;
	for (f_it = mesh.faces_sbegin(); minid > 0; minid--, f_it++) {}
	for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)//找到全部的点（包括邻域，面上的三个点，以及重复点）
	{
		for (auto vv_it = mesh.vv_iter(*fv_it); vv_it.is_valid(); ++vv_it)
		{
			xianglinpoint[i].x = mesh.point(*vv_it).data()[0];
			xianglinpoint[i].y = mesh.point(*vv_it).data()[1];
			xianglinpoint[i].z = mesh.point(*vv_it).data()[2];
			xianglinpoint[i].id = vv_it.handle().idx();
			i++;
		}
	}

	j = 0;
	for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {//所输入面的三个点
		point[j].x = mesh.point(*fv_it).data()[0];
		point[j].y = mesh.point(*fv_it).data()[1];
		point[j].z = mesh.point(*fv_it).data()[2];
		point[j].id = fv_it.handle().idx();
		j++;
	}

#pragma region 去掉重复点，找到邻域顶点
	for (k = 0; k < i; k++)
	{
		if (xianglinpoint[k].id == point[0].id)
		{
			for (l = k; l < i; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			i--;
		}
	}
	for (k = 0; k < i; k++)
	{
		if (xianglinpoint[k].id == point[1].id)
		{
			for (l = k; l < i; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			i--;
		}
	}
	for (k = 0; k < i; k++)
	{
		if (xianglinpoint[k].id == point[2].id)
		{
			for (l = k; l < i; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			i--;
		}
	}
	for (k = 0; k < i; k++)
	{
		for (l = k + 1; l < i; l++)
		{
			if (xianglinpoint[k].id == xianglinpoint[l].id)
			{
				for (int m = l; m < i; m++)
				{
					xianglinpoint[m].x = xianglinpoint[m + 1].x;
					xianglinpoint[m].y = xianglinpoint[m + 1].y;
					xianglinpoint[m].z = xianglinpoint[m + 1].z;
					xianglinpoint[m].id = xianglinpoint[m + 1].id;
				}
				i--;
			}
		}
	}
	for (j = 0; j < i; j++)
		std::cout << xianglinpoint[j].id << " "; /*<< "  " << xianglinpoint[j].x << " " << xianglinpoint[j].y << " " << xianglinpoint[j].z << std::endl;*/
	std::cout << std::endl;
#pragma endregion

#pragma region 排序

	double x = 0, y = 0;
	for (j = 0; j < i; j++)
	{
		x += xianglinpoint[j].x;
		y += xianglinpoint[j].y;
	}
	//center.x = (INT)x / i;
	//center.y = (INT)y / i;
	center.x = x / i;
	center.y = y / i;
	for (j = 0; j < i - 1; j++)
	{
		for (k = 0; k < i - j - 1; k++)
		{
			bool change;
			if (xianglinpoint[k].x >= 0 && xianglinpoint[k + 1].x < 0)
				change = true;
			else if (xianglinpoint[k].x == 0 && xianglinpoint[k + 1].x == 0)
				change = (xianglinpoint[k].y > xianglinpoint[k + 1].y);
			else
			{
				int det = (xianglinpoint[k].x - center.x) * (xianglinpoint[k + 1].y - center.y) - (xianglinpoint[k + 1].x - center.x) * (xianglinpoint[k].y - center.y);

				if (det < 0)
					change = true;
				else if (det > 0)
					change = false;
				else
				{
					double d1 = (xianglinpoint[k].x - center.x) * (xianglinpoint[k].x - center.x) + (xianglinpoint[k].y - center.y) * (xianglinpoint[k].y - center.y);
					double d2 = (xianglinpoint[k + 1].x - center.x) * (xianglinpoint[k + 1].x - center.y) + (xianglinpoint[k + 1].y - center.y) * (xianglinpoint[k + 1].y - center.y);
					change = (d1 > d2);
				}
			}

			if (change)
			{
				double tmp = xianglinpoint[k].x;
				xianglinpoint[k].x = xianglinpoint[k + 1].x;
				xianglinpoint[k + 1].x = tmp;

				tmp = xianglinpoint[k].y;
				xianglinpoint[k].y = xianglinpoint[k + 1].y;
				xianglinpoint[k + 1].y = tmp;

				tmp = xianglinpoint[k].z;
				xianglinpoint[k].z = xianglinpoint[k + 1].z;
				xianglinpoint[k + 1].z = tmp;

				int tmp1 = xianglinpoint[k].id;
				xianglinpoint[k].id = xianglinpoint[k + 1].id;
				xianglinpoint[k + 1].id = tmp1;
			}
		}
	}
	for (j = 0; j < i; j++)
		std::cout << xianglinpoint[j].id << " ";
	std::cout << std::endl;
#pragma endregion

	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();

	for (k = 0; k < 3; k++)
	{
		for (j = k + 1; j < 3; j++)
		{
			if (point[k].id >= point[j].id)
			{
				int tmp1 = point[k].id;
				point[k].id = point[j].id;
				point[j].id = tmp1;
			}
		}
	}
	
	Mesh::VertexIter v1, v2, v3;
	int id1, id2, id3;
	Mesh::VertexIter v_it1, v_it2, v_it3;
	id1 = point[0].id;
	id2 = point[1].id;
	id3 = point[2].id;
	for (v_it1 = mesh.vertices_sbegin(); id1 > 0; id1--, v_it1++) {}
	for (v_it2 = mesh.vertices_sbegin(); id2 > 0; id2--, v_it2++) {}
	for (v_it3 = mesh.vertices_sbegin(); id3 > 0; id3--, v_it3++) {}

	mesh.delete_vertex(v_it1);
	mesh.delete_vertex(v_it2);
	mesh.delete_vertex(v_it3);
	mesh.garbage_collection();

	Mesh::VertexHandle lianjie[20], zhongxin;
	
	point[3].x = (point[0].x + point[1].x + point[2].x) / 3.0;
	point[3].y = (point[0].y + point[1].y + point[2].y) / 3.0;
	point[3].z = (point[0].z + point[1].z + point[2].z) / 3.0;
	zhongxin = mesh.add_vertex(Mesh::Point(point[3].x, point[3].y, point[3].z));
	for (auto ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it)
		xianglinface++;
	
	Mesh::VertexIter v_lj1, v_lj2;
	//std::cout << xianglinpoint[0].id << std::endl;
	Mesh::VertexHandle zj1, zj2;
	std::cout << "i:" << i << std::endl;
	for (j = 0; j < i - 1; j++)
	{
		std::cout << "j:" << j << std::endl;
		bool v_1 = false, v_2 = false;
		zhuanhuan[0] = xianglinpoint[j];
		zhuanhuan[1] = xianglinpoint[j + 1];
		std::cout << zhuanhuan[0].id << " " << zhuanhuan[1].id << " ";
		for (v_lj1 = mesh.vertices_begin(); v_lj1 != mesh.vertices_end(); v_lj1++)
		{
			if (mesh.point(v_lj1).data()[0] == zhuanhuan[0].x&&mesh.point(v_lj1).data()[1] == zhuanhuan[0].y&&mesh.point(v_lj1).data()[2] == zhuanhuan[0].z)
			{
				v_1 = true;
				break;
			}
		}
		for (v_lj2 = mesh.vertices_begin(); v_lj2 != mesh.vertices_end(); v_lj2++)
		{
			if (mesh.point(v_lj2).data()[0] == zhuanhuan[1].x&&mesh.point(v_lj2).data()[1] == zhuanhuan[1].y&&mesh.point(v_lj2).data()[2] == zhuanhuan[1].z)
			{
				v_2 = true;
				break;
			}
		}
		if (!v_1)
			zj1 = mesh.add_vertex(Mesh::Point(zhuanhuan[0].x, zhuanhuan[0].y, zhuanhuan[0].z));
		if (!v_2)
			zj2 = mesh.add_vertex(Mesh::Point(zhuanhuan[1].x, zhuanhuan[1].y, zhuanhuan[1].z));
		if (v_1&&v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;
			mesh.add_face(v_lj1, v_lj2, zhongxin);
			mesh.garbage_collection();
		}
		else if (!v_1&&v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;
			mesh.add_face(zj1, v_lj2, zhongxin);
			mesh.garbage_collection();
		}
		else if (v_1 && !v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;
			mesh.add_face(v_lj1, zj2, zhongxin);
			mesh.garbage_collection();
		}
		else if(!v_1&&!v_2)
		{
			std::cout << v_1 << " " << v_2 << std::endl;;
			mesh.add_face(zj1, zj2, zhongxin);
			mesh.garbage_collection();
		}
	}

	bool v_1 = false, v_2 = false;
	zhuanhuan[0] = xianglinpoint[i-1];
	zhuanhuan[1] = xianglinpoint[0];
	for (v_lj1 = mesh.vertices_begin(); v_lj1 != mesh.vertices_end(); v_lj1++)
	{
		if (mesh.point(v_lj1).data()[0] == zhuanhuan[0].x&&mesh.point(v_lj1).data()[1] == zhuanhuan[0].y&&mesh.point(v_lj1).data()[2] == zhuanhuan[0].z)
		{
			v_1 = true;
			break;
		}
	}
	for (v_lj2 = mesh.vertices_begin(); v_lj2 != mesh.vertices_end(); v_lj2++)
	{
		if (mesh.point(v_lj2).data()[0] == zhuanhuan[1].x&&mesh.point(v_lj2).data()[1] == zhuanhuan[1].y&&mesh.point(v_lj2).data()[2] == zhuanhuan[1].z)
		{
			v_2 = true;
			break;
		}
	}
	if (!v_1)
		zj1 = mesh.add_vertex(Mesh::Point(zhuanhuan[0].x, zhuanhuan[0].y, zhuanhuan[0].z));
	if (!v_2)
		zj2 = mesh.add_vertex(Mesh::Point(zhuanhuan[1].x, zhuanhuan[1].y, zhuanhuan[1].z));
	if (v_1&&v_2)
	{
		std::cout << v_1 << " " << v_2 << std::endl;
		mesh.add_face(v_lj1, v_lj2, zhongxin);
		mesh.garbage_collection();
	}
	else if (!v_1&&v_2)
	{
		std::cout << v_1 << " " << v_2 << std::endl;
		mesh.add_face(zj1, v_lj2, zhongxin);
		mesh.garbage_collection();
	}
	else if (!v_1 && !v_2)
	{

		std::cout << v_1 << " " << v_2 << std::endl;;
		mesh.add_face(zj1, zj2, zhongxin);
		mesh.garbage_collection();
	}
	updateGL();
}

//指定某个面积简化
void InteractiveViewerWidget::vertex_Cluster(int id)
{
	
	int id1 = id, id2, id3;
	int i = 0, j = 0, k, l;
	int xianglinface = 0;
	Mesh::FaceIter f_it;
	Mesh::VertexHandle lianjie[20],zhongxin;
	//float dian[50][50];
	for (f_it = mesh.faces_sbegin(); id1 > 0; id1--, f_it++) {}
	//for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {//这个面的三个点
	//	xianglinpoint[i].x = mesh.point(*fv_it).data()[0];
	//	xianglinpoint[i].y = mesh.point(*fv_it).data()[1];
	//	xianglinpoint[i].z = mesh.point(*fv_it).data()[2];	
	//	//std::cout << i << "x :" << xianglinpoint[i].x << "y :" << xianglinpoint[i].y << "z :" << xianglinpoint[i].z << std::endl;
	//	
	//	i++;
	//}

	for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)//找到全部的点（包括邻域，面上的三个点，以及重复点）
	{
		for (auto vv_it = mesh.vv_iter(*fv_it); vv_it.is_valid(); ++vv_it)
		{
			//lianjie[i] = mesh.add_vertex(Mesh::Point(vv_it));
			xianglinpoint[i].x = mesh.point(*vv_it).data()[0];
			xianglinpoint[i].y = mesh.point(*vv_it).data()[1];
			xianglinpoint[i].z = mesh.point(*vv_it).data()[2];
			xianglinpoint[i].id = vv_it.handle().idx();
			//std::cout << i << " x :" << xianglinpoint[i].x << "y :" << xianglinpoint[i].y << "z :" << xianglinpoint[i].z << std::endl;
			i++;
		}
	}

	for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {//所输入面的三个点
		point[j].x = mesh.point(*fv_it).data()[0];
		point[j].y = mesh.point(*fv_it).data()[1];
		point[j].z = mesh.point(*fv_it).data()[2];
		point[j].id = fv_it.handle().idx();
		//std::cout << i << "x :" << xianglinpoint[i].x << "y :" << xianglinpoint[i].y << "z :" << xianglinpoint[i].z << std::endl;	
		j++;
	}

#pragma region 去掉重复点，找到邻域顶点
	for (k = 0; k < i; k++)
	{
		if (xianglinpoint[k].id==point[0].id)
		{
			for (l = k; l < i; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			i--;
		}
	}
	for (k = 0; k < i; k++)
	{
		if (xianglinpoint[k].id == point[1].id)
		{
			for (l = k; l < i; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			i--;
		}
	}
	for (k = 0; k < i; k++)
	{
		if (xianglinpoint[k].id == point[2].id)
		{
			for (l = k; l < i; l++)
			{
				xianglinpoint[l].x = xianglinpoint[l + 1].x;
				xianglinpoint[l].y = xianglinpoint[l + 1].y;
				xianglinpoint[l].z = xianglinpoint[l + 1].z;
				xianglinpoint[l].id = xianglinpoint[l + 1].id;
			}
			i--;
		}
	}
	for (k = 0; k < i; k++)
	{
		for (l = k + 1; l < i; l++)
		{
			if (xianglinpoint[k].id == xianglinpoint[l].id)
			{
				for (int m = l; m < i; m++)
				{
					xianglinpoint[m].x = xianglinpoint[m + 1].x;
					xianglinpoint[m].y = xianglinpoint[m + 1].y;
					xianglinpoint[m].z = xianglinpoint[m + 1].z;
					xianglinpoint[m].id = xianglinpoint[m + 1].id;
				}
				i--;
			}
		}
	}

	//for (k = 0; k < i; k++)
	//{
	//	for (j = k + 1; j < i; j++)
	//	{
	//		if (xianglinpoint[k].id >= xianglinpoint[j].id)
	//		{
	//			double tmp = xianglinpoint[k].x;
	//			xianglinpoint[k].x = xianglinpoint[j].x;
	//			xianglinpoint[j].x = tmp;

	//			tmp = xianglinpoint[k].y;
	//			xianglinpoint[k].y = xianglinpoint[j].y;
	//			xianglinpoint[j].y = tmp;

	//			tmp = xianglinpoint[k].z;
	//			xianglinpoint[k].z = xianglinpoint[j].z;
	//			xianglinpoint[j].z = tmp;

	//			int tmp1 = xianglinpoint[k].id;
	//			xianglinpoint[k].id = xianglinpoint[j].id;
	//			xianglinpoint[j].id = tmp1;
	//			//for (int z = 0; z<i; z++)
	//			//std::cout << xianglinpoint[z].id << "  ";
	//		}
	//		//std::cout << std::endl;
	//	}
	//}

	/*int min = xianglinpoint[0].x;
	int count = -1;
	for (j = 0; j < i; j++)
	{
		if (min == xianglinpoint[j].x)
			count++;
	}*/
	////std::cout << "count:" << count << std::endl;
	//for (j = 0; j <= count; j++)//0-count依次向上
	//{
	//	xianglin[j].id = xianglinpoint[j].id;
	//}

	//min = xianglinpoint[++count].x;
	//int count1 = -1;
	//for (j = 0; j < i; j++)
	//{
	//	if (min == xianglinpoint[j].x)
	//		count1++;
	//}
	//xianglin[count].id = xianglinpoint[count+count1].id;


	//std::cout << "count:" << count1 << std::endl;




	
#pragma endregion
	
	double x = 0, y = 0;
	for (j = 0; j < i; j++)
	{
		x += xianglinpoint[j].x;
		y += xianglinpoint[j].y;
	}
	center.x =(INT) x / i;
	center.y =(INT) y / i;
	std::cout << center.x << " " << center.y << std::endl;
	for (j = 0; j < i - 1; j++)
	{
		for (k = 0; k < i - j - 1; k++)
		{
			
			bool change;
			//若点k大于点k+1,即点k在点k+1顺时针方向,返回true,否则返回false
			if (xianglinpoint[k].x >= 0 && xianglinpoint[k + 1].x < 0)
				change = true;
			else if (xianglinpoint[k].x == 0 && xianglinpoint[k + 1].x == 0)
				change = (xianglinpoint[k].y > xianglinpoint[k + 1].y);
			else 
			{
				//向量Ok和向量Ok+1的叉积,根据向量叉积的定义，
				//向量OPi和OPj的叉积大于0，则向量OPj在向量OPi的逆时针方向，即点Pj小于点Pi。
				int det =(INT) (xianglinpoint[k].x - center.x) * (xianglinpoint[k + 1].y - center.y) - (xianglinpoint[k + 1].x - center.x) * (xianglinpoint[k].y - center.y);
				
				if (det < 0)
					change = true;
				else if (det > 0)
					change = false;
				else
				{
					double d1 = (xianglinpoint[k].x - center.x) * (xianglinpoint[k].x - center.x) + (xianglinpoint[k].y - center.y) * (xianglinpoint[k].y - center.y);
					double d2 = (xianglinpoint[k + 1].x - center.x) * (xianglinpoint[k + 1].x - center.y) + (xianglinpoint[k+1].y - center.y) * (xianglinpoint[k+1].y - center.y);
					change = (d1 > d2);
				}
			}

			if (change)
			{
				double tmp = xianglinpoint[k].x;
				xianglinpoint[k].x = xianglinpoint[k+1].x;
				xianglinpoint[k+1].x = tmp;

				tmp = xianglinpoint[k].y;
				xianglinpoint[k].y = xianglinpoint[k+1].y;
				xianglinpoint[k+1].y = tmp;

				tmp = xianglinpoint[k].z;
				xianglinpoint[k].z = xianglinpoint[k+1].z;
				xianglinpoint[k+1].z = tmp;

				int tmp1 = xianglinpoint[k].id;
				xianglinpoint[k].id = xianglinpoint[k+1].id;
				xianglinpoint[k+1].id = tmp1;
			}
			/*for (int m = 0; m<i; m++)
				std::cout << xianglinpoint[m].id << " ";
			std::cout << change << std::endl;*/
			
		}
	}

	for (k = 0; k < i; k++)
	{
		lianjie[k] = mesh.add_vertex(Mesh::Point(xianglinpoint[k].x, xianglinpoint[k].y, xianglinpoint[k].z));
	}

	//for(j=0;j<i;j++)//i是所输入面的邻域顶点的个数
	//	//std::cout << j << " x:" << xianglinpoint[j].x << " y:" << xianglinpoint[j].y << " z:" << xianglinpoint[j].z << std::endl;
	//	std::cout<<xianglinpoint[j].id<<std::endl;
	//std::cout << "i:" << i << std::endl;


	point[3].x = (point[0].x + point[1].x + point[2].x) / 3;//三个点的平均点
	point[3].y = (point[0].y + point[1].y + point[2].y) / 3;
	point[3].z = (point[0].z + point[1].z + point[2].z) / 3;
	//std::cout << "x:" << point[3].x << " y:" << point[3].y << " z:" << point[3].z;
	zhongxin = mesh.add_vertex(Mesh::Point(point[3].x, point[3].y, point[3].z));
	
	/*for (int m = 0; m < i; m++)
		std::cout << mesh.point(lianjie[m]).data()[0] << " " << mesh.point(lianjie[m]).data()[1] << " " << mesh.point(lianjie[m]).data()[2] << " ";*/
	std::cout << "i:" << i << std::endl;

	for (auto ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it)
		xianglinface++;

	if (xianglinface == 1 )
	{
		for (k = i - 1; k > 0; k--)//连接
		{
			mesh.add_face(lianjie[k], lianjie[k - 1], zhongxin);
			mesh.garbage_collection();
		}
	}

	else if (xianglinface == 3|| xianglinface == 2)
	{
		for (k = 0; k < i - 1; k++)
		{
			mesh.add_face(lianjie[k], lianjie[k + 1], zhongxin);
			mesh.garbage_collection();
		}
		mesh.add_face(lianjie[i-1], lianjie[0], zhongxin);
		mesh.garbage_collection();
	}
	
	

	//if (id == 0)
	//{
	//	for (k = 0; k < i - 1; k++)//连接
	//	{
	//		mesh.add_face(lianjie[k], lianjie[k + 1], zhongxin);
	//		mesh.garbage_collection();
	//	}
	//}
	//else if (id>=20&&id<=140&&(id-20)%2==0)
	//{
	//	for (k = 0; k < 2; k++)//连接
	//	{
	//		mesh.add_face(lianjie[k], lianjie[k + 1], zhongxin);
	//		mesh.garbage_collection();
	//	}
	//	mesh.add_face(lianjie[3], lianjie[0], zhongxin);
	//	mesh.garbage_collection();
	//	for (k = 3; k < 7; k=k+2)//连接
	//	{
	//		mesh.add_face(lianjie[k + 2], lianjie[k], zhongxin);
	//		mesh.garbage_collection();
	//	}
	//	mesh.add_face(lianjie[2], lianjie[4], zhongxin);
	//	mesh.garbage_collection();
	//	for (k = 4; k < 8; k = k + 2)//连接
	//	{
	//		mesh.add_face(lianjie[k], lianjie[k + 2], zhongxin);
	//		mesh.garbage_collection();
	//	}
	//	mesh.add_face(lianjie[8], lianjie[7], zhongxin);
	//	mesh.garbage_collection();
	//}
	
#pragma region 删除
	mesh.request_face_status();
	mesh.request_edge_status();
	mesh.request_vertex_status();

	for (k = 0; k < 3; k++)
	{
		for (j = k + 1; j < 3; j++)
		{
			if (point[k].id >= point[j].id)
			{
				int tmp1 = point[k].id;
				point[k].id = point[j].id;
				point[j].id = tmp1;
			}
		}
	}
	for (auto ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it)
	{
		mesh.delete_face(ff_it, true);
		mesh.garbage_collection();
	}

	Mesh::VertexIter v_it1, v_it2, v_it3;
	id1 = point[0].id;
	id2 = point[1].id;
	id3 = point[2].id;
	for (v_it1 = mesh.vertices_sbegin(); id1 > 0; id1--, v_it1++) {}
	for (v_it2 = mesh.vertices_sbegin(); id2 > 0; id2--, v_it2++) {}
	for (v_it3 = mesh.vertices_sbegin(); id3 > 0; id3--, v_it3++) {}
	mesh.delete_vertex(v_it1, true);
	mesh.garbage_collection();

	mesh.delete_vertex(v_it2, true);
	mesh.garbage_collection();

	mesh.delete_vertex(v_it3, true);
	mesh.garbage_collection();
#pragma endregion

//#pragma region 删除简化的面
//
//	mesh.request_face_status();
//	mesh.request_edge_status();
//	mesh.request_vertex_status();
//
//	/*id1 = id;
//	mesh::faceiter f_it_delete;
//	for (f_it_delete = mesh.faces_sbegin(); id1 > 0; id1--, f_it_delete++) {}
//	for (auto ff_it_delete = mesh.ff_iter(*f_it_delete); ff_it_delete.is_valid(); ++ff_it_delete)
//	{
//	mesh.delete_face(ff_it_delete, true);
//	mesh.garbage_collection();
//	}
//	mesh.delete_face(f_it_delete,true);
//	mesh.garbage_collection();*/
//	Mesh::VertexIter v_it;
//	id1 = point[0].id;
//	for (v_it = mesh.vertices_sbegin(); id1 > 0; id1--, v_it++) {}
//	mesh.delete_vertex(v_it, true);
//	mesh.garbage_collection();
//
//	id1 = point[1].id;
//	for (v_it = mesh.vertices_sbegin(); id1 > 0; id1--, v_it++) {}
//	mesh.delete_vertex(v_it, true);
//	mesh.garbage_collection();
//
//	id1 = point[2].id;
//	for (v_it = mesh.vertices_sbegin(); id1 > 0; id1--, v_it++) {}
//	mesh.delete_vertex(v_it, true);
//	mesh.garbage_collection();
//	for (j = 0; j < 3; j++)
//		std::cout << "x:" << point[j].x << " y:" << point[j].y << " z:" << point[j].z << " id:" << point[j].id << std::endl;
//#pragma endregion
	updateGL();
}

//查找一个顶点的一层领域顶点
void InteractiveViewerWidget::find_vv_by_id(int id)
{
	int i = 0;
	if (id < mesh.n_vertices() && id >= 0)
	{
		selectedVertex.push_back(id);
	}
	
	Mesh::VertexIter v_it;
	for ( v_it = mesh.vertices_sbegin(); id > 0; id--, v_it++){}
    for (auto vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
	{
		i = vv_it.handle().idx();
		std::cout <<"1ring顶点:"<< i <<std:: endl;
		if (i < mesh.n_vertices() && i>= 0)
		{
			selectedVertex.push_back(i);
		}
	}
	updateGL();
}

//查找一个顶点的一层领域面
void InteractiveViewerWidget::find_vf_by_id(int id)
{
	int i = 0;
	if (id < mesh.n_vertices() && id >= 0)
	{
		selectedVertex.push_back(id);
	}

	Mesh::VertexIter v_it;
	for (v_it = mesh.vertices_sbegin(); id > 0; id--, v_it++){}
	for (auto vf_it = mesh.vf_iter(*v_it); vf_it.is_valid(); ++vf_it)
	{
		i = vf_it.handle().idx();
		std::cout << "1ring面:" << i << std::endl;
		if (i < mesh.n_faces() && i >= 0)
		{
			selectedFace.push_back(i);
		}
	}
	updateGL();
}

//查找一个面的一层领域点
void InteractiveViewerWidget::find_fv_by_id(int id)
{
	int i = 0;
	if (id < mesh.n_faces() && id >= 0)
	{
		selectedFace.push_back(id);
	}

	Mesh::FaceIter f_it;
	for (f_it = mesh.faces_sbegin(); id > 0; id--, f_it++){}
	for (auto fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
	{
		i = fv_it.handle().idx();
		std::cout << "1ring点:" << i << std::endl;
		if (i < mesh.n_vertices() && i >= 0)
		{
			selectedVertex.push_back(i);
		}
	}
	updateGL();
}

//查找一个面的一层领域面
void InteractiveViewerWidget::find_ff_by_id(int id)
{
	int i = 0;
	if (id < mesh.n_faces() && id >= 0)
	{
		selectedFace.push_back(id);
	}

	Mesh::FaceIter f_it;
	for (f_it = mesh.faces_sbegin(); id > 0; id--, f_it++){}
	for (auto ff_it = mesh.ff_iter(*f_it); ff_it.is_valid(); ++ff_it)
	{
		i = ff_it.handle().idx();
		std::cout << "1ring面:" << i << std::endl;
		if (i < mesh.n_faces() && i >= 0)
		{
			selectedFace.push_back(i);
		}
	}
	updateGL();
}

//void InteractiveViewerWidget::find_vstar_by_id(int id)
//{
//	selectedEdge=find_vstar_by_id_without_update(id);
//	updateGL();
//}

//查找一个顶点的入射边
std::vector<int> InteractiveViewerWidget::find_incident_edges_of_vertices_without_update(int id)
{
	int i = 0;
	//Mesh::VertexIter v_it;
	std::vector<int> temp;
	if (id < mesh.n_vertices() && id >= 0)
	{
		selectedVertex.push_back(id);
	}
	auto v_handle=mesh.vertex_handle(id);
	//for (v_it = mesh.vertices_sbegin(); id > 0; id--, v_it++){}
	for (auto vstar_it = mesh.ve_iter(v_handle); vstar_it.is_valid(); ++vstar_it)
	{
		i = vstar_it.handle().idx();
		if (i < mesh.n_edges() && i >= 0)
		{
			temp.push_back(i);
		}
	}
	return temp;
}

//查找一组顶点的边界边
void InteractiveViewerWidget::find_boundary_edges_of_vertices(std::string ids)
{
	std::vector<int> vid_vector = get_ids(ids);
	bool* seleced_flags = set_vertex_flag(vid_vector);
	Mesh::HalfedgeHandle cur_he,start_he;
	bool b = true;
	for (auto it = vid_vector.begin(); it != vid_vector.end(); ++it)
	{
		if (*it < mesh.n_vertices() && *it >= 0)
		{
			selectedVertex.push_back(*it);
		}
		if (b)
			start_he = mesh.halfedge_handle(mesh.vertex_handle(*it));
	
		 if (b&&seleced_flags[mesh.to_vertex_handle(start_he).idx()]
			 && !seleced_flags[mesh.to_vertex_handle(mesh.next_halfedge_handle(start_he)).idx()])
		{
			 cur_he = start_he;
			 selectedEdge.push_back(mesh.edge_handle(start_he).idx());
			b = false;
		}
	}
	do{
		/*std::cout << "cur_he:" << mesh.edge_handle(cur_he).idx() << std::endl;
		std::cout << "from_vertex_handle" << mesh.from_vertex_handle(mesh.next_halfedge_handle(cur_he)).idx()<<std::endl;
		std::cout << "to_vertex_handle:" << mesh.to_vertex_handle(mesh.next_halfedge_handle(cur_he)).idx() << std::endl;*/
		do{
			cur_he = mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur_he));
		}  while  (!(seleced_flags[mesh.from_vertex_handle(mesh.next_halfedge_handle(cur_he)).idx()]
			&& seleced_flags[mesh.to_vertex_handle(mesh.next_halfedge_handle(cur_he)).idx()]));

		if (seleced_flags[mesh.from_vertex_handle(mesh.next_halfedge_handle(cur_he)).idx()]
				&& seleced_flags[mesh.to_vertex_handle(mesh.next_halfedge_handle(cur_he)).idx()]){
				selectedEdge.push_back(mesh.edge_handle(mesh.next_halfedge_handle(cur_he)).idx());
				cur_he =mesh.next_halfedge_handle(cur_he);
			}
	} while (start_he.idx() != cur_he.idx());

	updateGL();
}

#pragma region 以顶点id查领域边
//调用查找一个顶点的一层领域边并刷新
void InteractiveViewerWidget::find_ve_by_id(int id)
{
	find_ve_by_id_without_update(id);
	updateGL();
}

//查找一个顶点的一层领域边但不刷新视图
std::vector<int> InteractiveViewerWidget::find_ve_by_id_without_update(int id)
{
	int i = 0,t=id;
	if (id < mesh.n_vertices() && id >= 0)
	{
		selectedVertex.push_back(id);
	}
	Mesh::VertexIter v_it;
	
	for (v_it = mesh.vertices_sbegin(); t > 0; t--, v_it++){}
	for (auto ve_it = mesh.ve_iter(*v_it); ve_it.is_valid(); ++ve_it)
	{
		Mesh::HalfedgeHandle he_handle = mesh.halfedge_handle(*ve_it, 0);
		/*std::cout << "edge_handle:" << mesh.edge_handle(he_handle).idx() << std::endl;
		std::cout << "he_handle:" << he_handle.idx()  << std::endl;
		std::cout << "mesh.to_vertex_handle(he_handle).idx() :" << mesh.to_vertex_handle(he_handle).idx() << std::endl;
		std::cout << "mesh.from_vertex_handle(he_handle).idx() :" << mesh.from_vertex_handle(he_handle).idx() << std::endl;*/
		if (mesh.to_vertex_handle(he_handle).idx() == id)
		{//目的角是此顶点则取反向边的下一条边
			i = mesh.edge_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(he_handle))).idx();
			if (i < mesh.n_edges() && i >= 0)
			{
				selectedEdge.push_back(i);
			}
		}
		if (mesh.from_vertex_handle(he_handle).idx() == id)
		{
			i = mesh.edge_handle(mesh.next_halfedge_handle(he_handle)).idx();
			if (i < mesh.n_edges() && i >= 0)
			{
				selectedEdge.push_back(i);
			}
		}
	}
	selectedEdge.erase(unique(selectedEdge.begin(), selectedEdge.end()), selectedEdge.end());
	return selectedEdge;
}

//查找一组顶点的领域边
void InteractiveViewerWidget::find_1ring_edges_of_vertices(std::string ids)
{
	int i;
	std::vector<int> selectedEdgeTemp, vstar; //selectedHE,
	std::vector<int> vid_vector = get_ids(ids);
	if (vid_vector.size() > 1){
		for (auto vid_it = vid_vector.begin(); vid_it != vid_vector.end(); ++vid_it)
		{
			std::vector<int> ev = find_ve_by_id_without_update(*vid_it);
			std::vector<int> vstarTemp = find_incident_edges_of_vertices_without_update(*vid_it);
			vstar.insert(vstar.end(), vstarTemp.begin(), vstarTemp.end());
			selectedEdgeTemp.insert(selectedEdgeTemp.end(), ev.begin(), ev.end());
		}
		selectedEdgeTemp.insert(selectedEdgeTemp.end(), vstar.begin(), vstar.end());
		//去除边集中的重复元素
		//unique()函数将重复的元素放到vector的尾部 然后返回指向第一个重复元素的迭代器 再用erase函数擦除从这个元素到最后元素的所有的元素
		//sort(selectedEdgeTemp.begin(), selectedEdgeTemp.end());
		selectedEdgeTemp.erase(unique(selectedEdgeTemp.begin(), selectedEdgeTemp.end()), selectedEdgeTemp.end());
		vstar.erase(unique(vstar.begin(), vstar.end()), vstar.end());

		//将一层领域边和顶点的放射边集合加在一起后形成大集合，再大集合减去放射集
		for (auto es_it = vstar.begin(); es_it != vstar.end(); ++es_it)
		{
			for (auto ve_it = selectedEdgeTemp.begin(); ve_it != selectedEdgeTemp.end();)
			{
				if ((*es_it) == (*ve_it))
				{
					ve_it = selectedEdgeTemp.erase(ve_it);
				}
				else
				{
					++ve_it;
				}
			}
		}
		selectedEdge.clear();
		selectedEdge = selectedEdgeTemp;
		//std::vector<Mesh::Halfedge> halfedges =find_vse_by_ids_loop(set_vertex_flag(id_vector));

		//将边集转化为对应的半边集
		/*for (auto e = selectedEdgeTemp.begin(); e != selectedEdgeTemp.end(); ++e)
		{
		for (auto it = mesh.halfedges_begin(); it != mesh.halfedges_end(); ++it)
		{
		if (mesh.edge_handle(*it).idx() == *e)
		{
		selectedHE.push_back((*it).idx());
		}
		}
		std::cout << "selectedEdgeTemp2: " << *e << std::endl;
		}
		selectedHE.erase(unique(selectedHE.begin(), selectedHE.end()), selectedHE.end());*/


		//得到最终的边集
		//filtrate_edge_connective_with_vertices(selectedHE, vid_vector);
		//selectedEdge = filtrate_edge_connective_with_vertices(selectedHE, vid_vector);

		for (auto id : vid_vector)
		{
			if (id < mesh.n_vertices() && id >= 0)
			{
				selectedVertex.push_back(id);
			}
		}
		updateGL();
	}
	else
	{
		find_ve_by_id(*vid_vector.begin());
	}
}
#pragma endregion
//查找一个面的边
void InteractiveViewerWidget::find_1ring_edges_of_face(int id)
{
	selectedEdge = find_1ring_edges_of_face_without_update(id);
	updateGL();
}
std::vector<int> InteractiveViewerWidget::find_1ring_edges_of_face_without_update(int id)
{
	std::vector<int> temp;
	int i = 0;

	auto f_handle = mesh.face_handle(id);
	//for (v_it = mesh.vertices_sbegin(); id > 0; id--, v_it++){}
	for (auto fe_it = mesh.fe_iter(f_handle); fe_it.is_valid(); ++fe_it)
	{
		i = (*fe_it).idx();
		if (i < mesh.n_edges() && i >= 0)
		{
			temp.push_back(i);
		}
	}
	return temp;
}

//查找一组面的边界边
void InteractiveViewerWidget::find_edges_of_faces_by_ids(std::string ids)
{
	std::vector<int> f_ids = get_ids(ids);
	std::vector<int> e_vector;
	for (auto it = f_ids.begin(); it != f_ids.end(); ++it)
	{
		std::vector<int> t = find_1ring_edges_of_face_without_update(*it);
		e_vector.insert(e_vector.end(), t.begin(), t.end());
	}
		sort(e_vector.begin(), e_vector.end());
		int c = e_vector[0];
		std::cout << "e:" <<c<< std::endl;
		std::cout << "e:" << e_vector[1] << std::endl;
		for (int i = 1;i<e_vector.size(); i++)
		{
			std::cout << "e:"<<e_vector[i] << std :: endl;
			if (e_vector[i] ==c)
			{
				e_vector[i-1] = -1;
				e_vector[i] = -1;
			}
			else 
			{
				c = e_vector[i];
			}
		}
		for (auto it = e_vector.begin(); it != e_vector.end();)
		{
			if ((*it) == -1)
			{
				it = e_vector.erase(it);
			}
			else ++it;
		}
	selectedEdge = e_vector;
	updateGL();
}
bool* InteractiveViewerWidget::set_vertex_flag(std::vector<int> &selected_ids)
{
	int size = mesh.n_vertices();
	bool *selected_flags = new bool[size];
	for (int i = 0; i <mesh.n_vertices(); ++i)
	{
		*(selected_flags + i) = false;
	}

	for (auto v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		for (auto id_it = selected_ids.begin(); id_it != selected_ids.end(); ++id_it)
		{
			if (*id_it == v_it.handle().idx())
			{
				*(selected_flags + v_it.handle().idx()) = true;
			}
		}
	}
	return selected_flags;
}

//将输入的string类型提取数字返回
std::vector<int> InteractiveViewerWidget::get_ids(std::string ids_str)
{
	int i=0, j=0, d=0, sum=0,k=0;
	std::vector<int> selected_ids;
	for (i = 0; i < ids_str.length(); ++i, sum = 0)
	{
		j = i;
		while (1)
		{
			if (!isdigit(ids_str[i]))
			{
				break;
			}
			i++;
		}
		d = i - j - 1;
		while (d >= 0)
		{
			k = std::pow(10, d);
			std::stringstream ss;
			int t;
			ss << ids_str[j];
			ss >> t;
			sum +=t *k ;
			j++;
			d--;
		}
		selected_ids.push_back(sum);
	}
	return selected_ids;
}

//查找一条边的一层领域边
void InteractiveViewerWidget::find_1ring_edges_of_edge(int id)
{
	std::vector<int> ids;
	if (id < mesh.n_edges() && id >= 0)
	{
		selectedEdge.push_back(id);
		updateGL();
	}
	Mesh::HalfedgeHandle start, cur;
	start = mesh.halfedge_handle(mesh.edge_handle(id), 0);
	cur = start;

	while (start.idx() != mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur)).idx())
	{

		cur = mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur));
		//std::cout << "cur1:" << mesh.edge_handle(cur).idx() << std::endl;

		if (start.idx() != mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur)).idx())
			ids.push_back(mesh.edge_handle(mesh.prev_halfedge_handle(cur)).idx());

	}
	start = mesh.halfedge_handle(mesh.edge_handle(id), 1);
	cur = start;
	while (start.idx() != mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur)).idx())
	{
		cur = mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur));
		//std::cout << "cur1:" << mesh.edge_handle(cur).idx() << std::endl;
		if (start.idx() != mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur)).idx())
			ids.push_back(mesh.edge_handle(mesh.prev_halfedge_handle(cur)).idx());
	}
	//std::cout << "esize:" << selectedEdge.size() << std::endl;
	selectedEdge = ids;
	updateGL();
}
//查找边界
void InteractiveViewerWidget::find_all_boundary_edge()
{
	std::vector<std::vector<int>> all_boundary;
	std::vector<int> hole_ids;
	std::vector<Mesh::HalfedgeHandle> hole;
	Mesh::HalfedgeHandle start, cur;
	/*for (int i = 0; i < mesh.n_edges(); ++i)
	{
	if (mesh.is_boundary(mesh.edge_handle(i)))
	{
	bool exists = false;
	for (auto it = all_boundary.begin(); it != all_boundary.end(); ++it)
	{
	auto it_t = std::find((*it).begin(), (*it).end(), mesh.edge_handle(i));

	if (it_t != (*it).end())
	{
	exists = true;
	}
	}
	if (!exists)
	{
	std::vector<Mesh::EdgeHandle> hole;
	hole.push_back(mesh.edge_handle(i));

	all_boundary.push_back(hole);
	}
	}
	}*/
	bool* heFlag = new bool[mesh.n_halfedges()];
	for (int i = 0; i < mesh.n_halfedges(); i++)
	{
		*(heFlag + i) = false;
	}
	for (int i = 0; i < mesh.n_halfedges(); ++i)
	{
		start = mesh.halfedge_handle(i);
		cur = start;
		if (mesh.is_boundary(cur) && !heFlag[i])
		{
			heFlag[i] = true;
			hole.push_back(cur);
			do{
				while (!mesh.is_boundary(mesh.next_halfedge_handle(cur)))
				{
					cur = mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(cur));
				}
				if (!heFlag[mesh.next_halfedge_handle(cur).idx()])
				{
					cur = mesh.next_halfedge_handle(cur);
					heFlag[cur.idx()] = true;
					hole.push_back(cur);

				}
				else break;
			} while (start.idx() != cur.idx());
		}
	}
	if (start.idx() == cur.idx())
	{
		for (int i = 0; i < hole.size(); ++i)
		{
			hole_ids.push_back(mesh.edge_handle(mesh.halfedge_handle(hole[i].idx())).idx());
		}
	}
	all_boundary.push_back(hole_ids);
}

//查找边的分割区域
void InteractiveViewerWidget::split_by_edges(std::string ids)
{
	std::vector<int> id_vector = get_ids(ids);
	bool* edgesFlag = set_edge_flag(id_vector);
	bool* heFlag = new bool[mesh.n_halfedges()];
	bool* faceFlag = new bool[mesh.n_faces()];
	for (int i = 0; i <mesh.n_halfedges(); ++i)
	{
		*(heFlag + i) = false;
	}
	for (int i = 0; i <mesh.n_faces(); ++i)
	{
		*(faceFlag + i) = false;
	}
	std::vector<std::vector<Mesh::FaceHandle>> all;
	for (int i = 0; i < mesh.n_halfedges(); ++i)
	{
		if (!heFlag[i])
		{
			std::vector<Mesh::FaceHandle> face_vector;
			std::stack<Mesh::HalfedgeHandle> he_stack;
			he_stack.push(mesh.halfedge_handle(i));
			while (!he_stack.empty())
			{
				Mesh::HalfedgeHandle cur = he_stack.top();
				heFlag[cur.idx()] = true;
				if (!faceFlag[mesh.face_handle(cur).idx()])
				{
					face_vector.push_back(mesh.face_handle(cur));
					faceFlag[mesh.face_handle(cur).idx()];
				}
				std::vector<Mesh::HalfedgeHandle> he_vector_temp;
				he_vector_temp.push_back(cur);
				for (auto it = he_vector_temp.begin(); it != he_vector_temp.end(); ++it)
				{
					if (!heFlag[(*it).idx()]
						&& mesh.face_handle(*it).idx() != -1
						&& edgesFlag[mesh.edge_handle(*it).idx()])
					{
						he_stack.push(*it);
					}
				}
			}
			if (!face_vector.empty())
			{
				all.push_back(face_vector);
			}
		}
	}
	for (auto it1 = all.begin(); it1 != all.end(); ++it1)
	{
		for (auto it2 = (*it1).begin(); it2 != (*it1).end(); ++it2)
		{
			selectedFace.push_back((*it2).idx());
		}
	}
	updateGL();
}

bool* InteractiveViewerWidget::set_edge_flag(std::vector<int> selected_ids)
{
	bool* selected_edge = new bool(mesh.n_edges());

	for (int i = 0; i < mesh.n_edges(); i++)
	{
		*(selected_edge + i) = false;
		for (auto it = selected_ids.begin(); it != selected_ids.end(); ++it)
		{
			if (mesh.edge_handle(i).idx() == mesh.edge_handle(*it).idx())
			{
				selected_edge[i] = true;
			}
		}
	}
	return selected_edge;
}
#pragma region abandoned
/*


std::vector<int> InteractiveViewerWidget::filtrate_edge_connective_with_vertices(std::vector<int> &heid_vector, std::vector<int> &vid_vector)
{
int i, j, k;
bool is_in_ev = false;
//在mesh的所有顶点中标记选择的顶点
bool *set_flags= set_vertex_flag(vid_vector);
Mesh::VertexHandle start_v_handle, current_v_handle;
Mesh::HalfedgeHandle start_he_handle, current_he_handle;
std::vector<int> selectedE;

for (i = 0; i < vid_vector.size(); ++i)
{
start_v_handle = mesh.vertex_handle(vid_vector[i]);
current_v_handle = start_v_handle;
start_he_handle = mesh.halfedge_handle(start_v_handle);
current_he_handle = start_he_handle;

do{
for (j = 0; j < heid_vector.size(); ++j)
{
//判断是否在半边集中
if (mesh.edge_handle(current_he_handle).idx() == heid_vector[j])
{
//还要判断头尾是否在顶点集中
if (set_flags[mesh.to_vertex_handle(current_he_handle).idx()]
|| set_flags[mesh.from_vertex_handle(current_he_handle).idx()])
{
//头尾有一个在顶点集中就删掉这条边
heid_vector.erase(heid_vector.begin() + j);
is_in_ev = true;
break;
}
}
}

if (!is_in_ev)
{
//在半边集中则将当前半边设为上一条的反向的next
current_he_handle = mesh.next_halfedge_handle(mesh.opposite_halfedge_handle((current_he_handle)));
}
else
{
//否则将当前边设为上一条的next
current_he_handle = mesh.next_halfedge_handle(current_he_handle);
is_in_ev = false;
}
} while (start_v_handle.idx() != current_v_handle.idx());
}

for (auto it = heid_vector.begin(); it != heid_vector.end(); ++it)
{
std::cout << "mesh.edge_handle(mesh.halfedge_handle(*it)).idx():" << mesh.edge_handle(mesh.halfedge_handle(*it)).idx() << std::endl;
selectedEdge.push_back(mesh.edge_handle(mesh.halfedge_handle(*it)).idx());
}
selectedEdge.erase(unique(selectedEdge.begin(), selectedEdge.end()), selectedEdge.end());
return selectedEdge;
}
std::vector<Mesh::Halfedge> InteractiveViewerWidget::find_vse_by_ids_loop(int *selected_flags)
{
	std::vector<Mesh::Halfedge> boundaryHFs;
	auto start_he_handle = GetStart(selected_flags);
	auto current_he_handle = start_he_handle;
	int from_vertex_idx;
	int to_vertex_idx;
	
	assert(mesh.is_valid_handle(current_he_handle));
	do
	{
		do
		{
			if (!selected_flags[mesh.from_vertex_handle(current_he_handle).idx()])
			{
				boundaryHFs.push_back(mesh.halfedge(mesh.prev_halfedge_handle(current_he_handle)));
			
			}
			current_he_handle = mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(current_he_handle));
			
			from_vertex_idx = mesh.from_vertex_handle(current_he_handle).idx();
			to_vertex_idx = mesh.to_vertex_handle(current_he_handle).idx();
		} while (!selected_flags[from_vertex_idx] &&
			selected_flags[to_vertex_idx]);
		do
		{
			from_vertex_idx = mesh.from_vertex_handle(current_he_handle).idx();
			to_vertex_idx = mesh.to_vertex_handle(current_he_handle).idx();
			current_he_handle = mesh.next_halfedge_handle(current_he_handle);
			
		} while (mesh.is_boundary(current_he_handle) &&
			selected_flags[from_vertex_idx] &&
			selected_flags[to_vertex_idx]);
		std::cout << "current_he_handle:" << current_he_handle.idx() << "  start_he_handle:" << start_he_handle.idx() << std::endl;
	} while (current_he_handle.idx() != start_he_handle.idx());
	return boundaryHFs;
}
Mesh::HalfedgeHandle InteractiveViewerWidget::GetStart(int *selected_flags)
{
	for (auto he_it = mesh.halfedges_begin(); he_it != mesh.halfedges_end();++he_it)
	{
		auto from_vertex_idx = mesh.from_vertex_handle(*he_it).idx();
		auto to_vertex_idx = mesh.to_vertex_handle(*he_it).idx();
		auto next_vertex_idx = mesh.to_vertex_handle(mesh.next_halfedge_handle(*he_it)).idx();
		std::cout << "he_it:" << (*he_it).idx()<< "  from_vertex_idx:" << from_vertex_idx << "  to_vertex_idx：" << to_vertex_idx << "  next_vertex_idx" << next_vertex_idx << std::endl;
		if (selected_flags[from_vertex_idx] && selected_flags[to_vertex_idx] && !selected_flags[next_vertex_idx])
		{
			assert(mesh.is_valid_handle(*he_it));

			return *he_it;
		}
	}
	return Mesh::HalfedgeHandle();
}*/
#pragma endregion
