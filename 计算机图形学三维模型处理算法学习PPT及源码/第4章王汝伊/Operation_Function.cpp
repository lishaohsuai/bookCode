/*
wry
*/

#include "InteractiveViewerWidget.h"

//面添加
void InteractiveViewerWidget::faceAdd(int x, int y)
{
	int r = find_vertex_using_selected_point();
	Mesh::VertexHandle v = mesh.vertex_handle(r);
	if (v.idx() < 0)
	{
		Mesh::VertexHandle vertexHandle = mesh.add_vertex(Mesh::Point(selectedPoint[0], selectedPoint[1], selectedPoint[2]));
		verticesAdd.push_back(vertexHandle);
	}
	else
	{
		verticesAdd.push_back(v);
	}
	
	if (verticesAdd.size() == 3)
	{		
		//Mesh::FaceHandle face = mesh.add_face(v1, v2, v3);
		Mesh::FaceHandle face = addFace(verticesAdd);
		int faceAdd = face.idx();
		if (faceAdd >= 0)
		{
			facesAdd.push_back(faceAdd);
		}

		verticesAdd.clear();
		mesh.update_normals();
		updateGL();
		//setDrawMode(InteractiveViewerWidget::FLAT_POINTS);
		//setMouseMode(InteractiveViewerWidget::TRANS);
	}

}

//面删除
void InteractiveViewerWidget::faceDelete(int x, int y)
{
	facesAdd.clear();
	int deleteFace = find_face_using_selected_point();
	if (deleteFace < 0) return;

	Mesh::FaceHandle faceHandle = mesh.face_handle(deleteFace);

	//mesh.delete_face(faceHandle);//底层已实现 半边对应的面设为NULL  
	removeFace(faceHandle);
	mesh.garbage_collection();
}

//边删除
void InteractiveViewerWidget::edgeDelete(int x, int y)
{
	int deleteEdge = find_edge_using_selected_point();
	if (deleteEdge < 0) return;

	Mesh::EdgeHandle edgeHandle = mesh.edge_handle(deleteEdge);
	//mesh.delete_edge(edgeHandle);
	removeEdge(edgeHandle);
	mesh.garbage_collection();
}

//点删除
void InteractiveViewerWidget::vertexDelete(int x, int y)
{
	int deleteVertex = find_vertex_using_selected_point();
	if (deleteVertex < 0) return;
	Mesh::VertexHandle vertexHandle = mesh.vertex_handle(deleteVertex);
	//mesh.delete_vertex(vertexHandle);
	removeVertex(vertexHandle);
	mesh.garbage_collection();
}

//点分割
void InteractiveViewerWidget::vertexSplit(int x, int y)
{
	if (verticesSplit.size() < 3)
	{
		int r = find_vertex_using_selected_point();
		Mesh::VertexHandle vertexHandle = mesh.vertex_handle(r);
		verticesSplit.push_back(vertexHandle);
	}
	else if (verticesSplit.size() == 3)
	{
		Mesh::VertexHandle vertexHandle = mesh.add_vertex(Mesh::Point(selectedPoint[0], selectedPoint[1], selectedPoint[2]));
		verticesSplit.push_back(vertexHandle);
	}
	else if (verticesSplit.size() == 4)
	{
		mesh.set_point(verticesSplit[0], Mesh::Point(selectedPoint[0], selectedPoint[1], selectedPoint[2]));

		//改变半边结构关系
		Mesh::HalfedgeHandle he = mesh.find_halfedge(verticesSplit[0], verticesSplit[1]);
		std::vector<Mesh::HalfedgeHandle> tempHalfedgeHandle;
		std::vector<Mesh::VertexHandle> tempPoints;		//方法一   遍历半边 删面 重新加面
		tempHalfedgeHandle.push_back(he);
		tempPoints.push_back(verticesSplit[1]);			//方法一
		while (mesh.to_vertex_handle(he) != verticesSplit[2])
		{
			he = mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(he));
			tempHalfedgeHandle.push_back(he);//存储半边
			//mesh.delete_face(mesh.face_handle(he));		//方法一
			tempPoints.push_back(mesh.to_vertex_handle(he)); //方法一
		}

		//mesh.garbage_collection();



		//方法一
		for (int i = 0; i < tempPoints.size() - 1; i++)
		{
			//mesh.add_face(verticesSplit[3], tempPoints[i + 1], tempPoints[i]);
		}

		//mesh.add_face(verticesSplit[3], verticesSplit[1], verticesSplit[0]);
		////Mesh::Point v = (mesh.point(verticesSplit[0]) + mesh.point(verticesSplit[3]) + mesh.point(verticesSplit[1])) / 3.0;
		////Mesh::VertexHandle vh = mesh.add_vertex(v);

		////mesh.add_face(verticesSplit[0], vh, verticesSplit[1]);
		////mesh.add_face(verticesSplit[0], verticesSplit[3], vh);
		////mesh.add_face(vh, verticesSplit[3], verticesSplit[1]);
		//mesh.add_face(verticesSplit[3], verticesSplit[0], verticesSplit[2]);

		//方法二  参照书
		for (int i = 0; i < tempHalfedgeHandle.size() - 1; i++)
		{
			mesh.set_vertex_handle(mesh.opposite_halfedge_handle(tempHalfedgeHandle[i]), verticesSplit[3]);
		}

		Mesh::HalfedgeHandle v2s1 = mesh.new_edge(verticesSplit[3], verticesSplit[1]);
		Mesh::HalfedgeHandle s1v1 = mesh.opposite_halfedge_handle(v2s1);

		mesh.set_vertex_handle(s1v1, verticesSplit[0]);

		Mesh::HalfedgeHandle v1v2 = mesh.new_edge(verticesSplit[0], verticesSplit[3]);
		Mesh::HalfedgeHandle v2v1 = mesh.opposite_halfedge_handle(v1v2);

		mesh.set_next_halfedge_handle(s1v1, v1v2);
		mesh.set_next_halfedge_handle(v1v2, v2s1);
		mesh.set_next_halfedge_handle(v2s1, s1v1);

		Mesh::HalfedgeHandle v1s1 = mesh.find_halfedge(verticesSplit[0], verticesSplit[1]);
		Mesh::FaceHandle f1 = mesh.new_face();

		mesh.set_face_handle(s1v1, f1);
		mesh.set_face_handle(v1v2, f1);
		mesh.set_face_handle(v2s1, f1);

		mesh.set_halfedge_handle(f1, v1v2);
		
	
		Mesh::HalfedgeHandle s2v2 = mesh.new_edge(verticesSplit[2], verticesSplit[3]);
		Mesh::HalfedgeHandle v2s2 = mesh.opposite_halfedge_handle(s2v2);

		Mesh::HalfedgeHandle v1s2 = mesh.find_halfedge(verticesSplit[0], verticesSplit[2]);
		Mesh::HalfedgeHandle prev = mesh.prev_halfedge_handle(v1s2);
		Mesh::HalfedgeHandle next = mesh.next_halfedge_handle(v1s2);

		mesh.set_next_halfedge_handle(prev, v2s2);
		mesh.set_next_halfedge_handle(v2s2, next);

		mesh.set_next_halfedge_handle(v2v1, v1s2);
		mesh.set_next_halfedge_handle(v1s2, s2v2);
		mesh.set_next_halfedge_handle(s2v2, v2v1);

		Mesh::FaceHandle f2 = mesh.new_face();
		mesh.set_face_handle(v1s2, f2);
		mesh.set_face_handle(s2v2, f2);
		mesh.set_face_handle(v2v1, f2);

		mesh.set_halfedge_handle(f2, v2v1);

		mesh.set_halfedge_handle(verticesSplit[3], v2v1);
		mesh.set_halfedge_handle(verticesSplit[0], v1v2);//放上面 有问题  索引查找为-1


		Mesh::HalfedgeHandle test0, test1;
		Mesh::EdgeHandle test;
		Mesh::VertexHandle p;
		for (int i = 0; i < tempPoints.size(); i++)
		{
			test0 = mesh.find_halfedge(verticesSplit[0], tempPoints[i]);
			test = mesh.edge_handle(test0);
			test0 = mesh.halfedge_handle(test, 0);
			test1 = mesh.halfedge_handle(test, 1);
			p = mesh.to_vertex_handle(test1);
			int j = 0;
		}



		//vertex_split(verticesSplit[3], verticesSplit[0], verticesSplit[2], verticesSplit[1]);//方法三  参照openmesh三角形网格分割
		//方法四  边遍历 重构v2与周围顶点的边  替换v1与周围的边

		verticesSplit.clear();
		mesh.update_normals();
		updateGL();
	}
}

//边合并
void InteractiveViewerWidget::edgeMerge(int x, int y)
{
	int desired_edge = find_edge_using_selected_point();
	if (desired_edge < 0) return;
	//方法一  参照书
	Mesh::EdgeHandle eh = mesh.edge_handle(desired_edge);
	if (!is_merge_ok(eh))//判断边能否合并
	{
		return;
	}

	Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(eh, 0);
	Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(eh, 1);

	Mesh::VertexHandle v0 = mesh.from_vertex_handle(h0);
	Mesh::VertexHandle v1 = mesh.to_vertex_handle(h0);


	Mesh::Point p = (mesh.point(v0) + mesh.point(v1)) / 2.0;
	mesh.set_point(v0, p);

	//将所有指向v1的顶点指向v0
	for (auto vih_it(mesh.vih_iter(v1)); vih_it.is_valid(); ++vih_it)
	{
		mesh.set_vertex_handle(*vih_it, v0);
	}

	//合并半边侧的面
	mergeHalfedge(h0);
	mergeHalfedge(h1);

	mesh.set_isolated(v1);
	mesh.status(v1).set_deleted(true);
	mesh.status(eh).set_deleted(true);

	mesh.garbage_collection();

	//方法二   参照openMesh已实现的collapse
	//方法三   删除面  重构面
	buildIndex();
}

//边逆时针旋转90
void InteractiveViewerWidget::edgeSwap(int x, int y)
{
	int desired_edge = find_edge_using_selected_point();
	if (desired_edge < 0) return;
	Mesh::EdgeHandle eh = mesh.edge_handle(desired_edge);
	if (mesh.is_boundary(eh))
	{
		return;
	}

	Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(eh, 0);
	Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 1);

	Mesh::VertexHandle vh0 = mesh.to_vertex_handle(heh0);
	Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh1);

	Mesh::HalfedgeHandle prev0 = mesh.prev_halfedge_handle(heh0);
	Mesh::HalfedgeHandle next0 = mesh.next_halfedge_handle(heh0);

	Mesh::HalfedgeHandle prev1 = mesh.prev_halfedge_handle(heh1);
	Mesh::HalfedgeHandle next1 = mesh.next_halfedge_handle(heh1);


	//逆时针旋转
	mesh.set_halfedge_handle(vh0, next0);
	mesh.set_halfedge_handle(vh1, next1);

	mesh.set_vertex_handle(heh0, mesh.to_vertex_handle(next0));
	mesh.set_vertex_handle(heh1, mesh.to_vertex_handle(next1));

	mesh.set_face_handle(next1, mesh.face_handle(heh0));
	mesh.set_face_handle(next0, mesh.face_handle(heh1));

	mesh.set_halfedge_handle(mesh.face_handle(heh0), heh0);
	mesh.set_halfedge_handle(mesh.face_handle(heh1), heh1);

	mesh.set_next_halfedge_handle(prev0, next1);
	mesh.set_next_halfedge_handle(next1, heh0);
	mesh.set_next_halfedge_handle(heh0, prev0);

	mesh.set_next_halfedge_handle(prev1, next0);
	mesh.set_next_halfedge_handle(next0, heh1);
	mesh.set_next_halfedge_handle(heh1, prev1);

}

//生成噪声
void InteractiveViewerWidget::addNoise(double threshold)
{
	srand((unsigned)time(NULL));
	double avgLength = 0.0;
	for (auto it = mesh.edges_begin(); it != mesh.edges_end(); it++)
	{
		Mesh::Scalar scalar = mesh.calc_edge_length(it.handle());
		avgLength += scalar;
	}

	avgLength /= mesh.n_edges();

	threshold *= avgLength;

	for (auto it = mesh.vertices_begin(); it != mesh.vertices_end(); it++)
	{
		Mesh::Normal normal = mesh.normal(it.handle());
		double scale = threshold * (rand() - 0.5f);
		//double scale = it.handle().idx() / 2000;
		mesh.set_point(it.handle(), Mesh::Point(mesh.point(it.handle()) + normal * scale));
	}

}

void InteractiveViewerWidget::drawFacesAdd()
{
	if (facesAdd.size() > 0)
	{
		glColor3f(1.0, 0.5, 1.0);
		Mesh::Point p;
		Mesh::ConstFaceVertexIter fv_it;
		Mesh::FaceHandle f_handle;
		for (unsigned int i = 0; i < facesAdd.size(); ++i)
		{
			f_handle = mesh.face_handle(facesAdd[i]);
			fv_it = mesh.fv_iter(f_handle);
			glBegin(GL_POLYGON);
			for (fv_it; fv_it; ++fv_it)
			{
				glVertex3dv(&mesh.point(fv_it)[0]);
			}
			glEnd();
		}

	}
}

Mesh::FaceHandle InteractiveViewerWidget::addFace(std::vector<Mesh::VertexHandle> _vhs)
{
	std::vector<Mesh::VertexHandle> vhVec(_vhs);

	bool usedVec[3];

	if (vhVec.size() < 3)
	{
		return Mesh::InvalidFaceHandle;
	}

	std::vector<Mesh::HalfedgeHandle> hehVec;
	for (int i = 0; i < vhVec.size(); i++)
	{
		if (!mesh.is_boundary(vhVec[i]))
		{
			return Mesh::InvalidFaceHandle;
		}
		Mesh::HalfedgeHandle heh = mesh.find_halfedge(vhVec[i], vhVec[(i + 1) % vhVec.size()]);
		
		if (heh.is_valid() && !mesh.is_boundary(heh))
		{
			return Mesh::InvalidFaceHandle;
		}

		hehVec.push_back(heh);
		usedVec[i] = mesh.halfedge_handle(vhVec[i]).is_valid();
	}

	Mesh::FaceHandle fh = mesh.new_face();

	for (int i = 0; i < hehVec.size(); i++)
	{
		//半边不存在,则两点之间肯定不存在边
		if (!hehVec[i].is_valid())
		{
			Mesh::HalfedgeHandle heh = mesh.new_edge(vhVec[i], vhVec[(i + 1) % vhVec.size()]);
			hehVec[i] = heh;
		}
		mesh.set_face_handle(hehVec[i], fh);
	}

	for (int i = 0; i < hehVec.size(); i++)
	{
		int j = (i + 1) % hehVec.size();
		connectHalfedge(hehVec[i], hehVec[j], usedVec[j]);
	}

	mesh.set_halfedge_handle(fh, hehVec[0]);

	std::vector<Mesh::VertexHandle>::iterator itV = vhVec.begin();
	for (; itV != vhVec.end(); itV++)
	{
		mesh.adjust_outgoing_halfedge(*itV);
	}

	return fh;
}

void InteractiveViewerWidget::removeFace(Mesh::FaceHandle _fh)
{
	mesh.status(_fh).set_deleted(true);

	//存储要被删的边
	std::vector<Mesh::EdgeHandle> deleted_edges;
	deleted_edges.reserve(3);

	//存储被删面的点
	std::vector<Mesh::VertexHandle>  vhandles;
	vhandles.reserve(3);

	Mesh::HalfedgeHandle hh;
	for (Mesh::FaceHalfedgeIter fh_it(mesh.fh_iter(_fh)); fh_it.is_valid(); ++fh_it)
	{
		hh = *fh_it;

		mesh.set_boundary(hh);

		if (mesh.is_boundary(mesh.opposite_halfedge_handle(hh)))
			deleted_edges.push_back(mesh.edge_handle(hh));  //边上的两条半边都是边界边

		vhandles.push_back(mesh.to_vertex_handle(hh));
	}

	//对边进行处理
	if (deleted_edges.size() > 0)
	{
		std::vector<Mesh::EdgeHandle>::iterator it = deleted_edges.begin();
		for (; it != deleted_edges.end(); it++)
		{
			Mesh::EdgeHandle eh = *it;
			Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(eh, 0);
			Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 1);

			Mesh::HalfedgeHandle prev0 = mesh.prev_halfedge_handle(heh0);
			Mesh::HalfedgeHandle next0 = mesh.next_halfedge_handle(heh0);
			Mesh::VertexHandle   v0 = mesh.to_vertex_handle(heh0);

			Mesh::HalfedgeHandle prev1 = mesh.prev_halfedge_handle(heh1);
			Mesh::HalfedgeHandle next1 = mesh.next_halfedge_handle(heh1);
			Mesh::VertexHandle   v1 = mesh.to_vertex_handle(heh1);

			//设置前后边的连接关系
			mesh.set_next_halfedge_handle(prev0, next1);
			mesh.set_next_halfedge_handle(prev1, next0);

			mesh.status(eh).set_deleted(true);

			mesh.status(heh0).set_deleted(true);
			mesh.status(heh1).set_deleted(true);

			//判断游离点
			if (mesh.halfedge_handle(v0) == heh1)
			{
				if (next0 == heh1)
				{
					mesh.status(v0).set_deleted(true);
					mesh.set_isolated(v0);
				}
				else
				{
					mesh.set_halfedge_handle(v0, next0);
				}
			}

			if (mesh.halfedge_handle(v1) == heh0)
			{
				if (next1 == heh0)
				{
					mesh.status(v1).set_deleted(true);
					mesh.set_isolated(v1);
				}
				else
				{
					mesh.set_halfedge_handle(v1, next1);
				}
			}
		}
	}

	//调整点的外向边
	std::vector<Mesh::VertexHandle>::iterator itV = vhandles.begin();
	for (; itV != vhandles.end(); itV++)
	{
		mesh.adjust_outgoing_halfedge(*itV);
	}


}

void InteractiveViewerWidget::removeEdge(Mesh::EdgeHandle _eh)
{
	Mesh::FaceHandle f0 = mesh.face_handle(mesh.halfedge_handle(_eh, 0));
	Mesh::FaceHandle f1 = mesh.face_handle(mesh.halfedge_handle(_eh, 1));

	if (f0.is_valid())
	{
		removeFace(f0);
	}
	if (f1.is_valid())
	{
		removeFace(f1);
	}
	//删除边和半边
	mesh.status(_eh).set_deleted(true);
	mesh.status(mesh.halfedge_handle(_eh, 0)).set_deleted(true);
	mesh.status(mesh.halfedge_handle(_eh, 1)).set_deleted(true);
}

void InteractiveViewerWidget::removeVertex(Mesh::VertexHandle _vh)
{
	std::vector<Mesh::FaceHandle> fHandles;
	for (auto it = mesh.vf_iter(_vh); it.is_valid(); it++)
	{
		fHandles.push_back(*it);
	}

	for (auto it = fHandles.begin(); it != fHandles.end(); it++)
	{
		removeFace(*it);
	}

	mesh.status(_vh).set_deleted(true);
}

void InteractiveViewerWidget::connectHalfedge(Mesh::HalfedgeHandle _heh1, Mesh::HalfedgeHandle _heh2, bool uesdVertex)
{
	Mesh::HalfedgeHandle cur = _heh1;
	Mesh::HalfedgeHandle next = _heh2;

	bool curIsNew, nextIsNew;
	if (mesh.next_halfedge_handle(cur).is_valid())
	{
		curIsNew = false;
	}
	else
	{
		curIsNew = true;
	}

	if (mesh.prev_halfedge_handle(next).is_valid())
	{
		nextIsNew = false;
	}
	else
	{
		nextIsNew = true;
	}

	if (curIsNew && nextIsNew) //说明边是新建的
	{
		if (uesdVertex)
		{
			Mesh::VertexHandle vh = mesh.to_vertex_handle(cur);
			Mesh::HalfedgeHandle close = mesh.halfedge_handle(vh);
			Mesh::HalfedgeHandle open = mesh.prev_halfedge_handle(close);

			mesh.set_next_halfedge_handle(open, mesh.opposite_halfedge_handle(cur));
			mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(next), close); 
		}
		else
		{
			mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(next), mesh.opposite_halfedge_handle(cur));
		}

	}
	else if (curIsNew && !nextIsNew)
	{
		mesh.set_next_halfedge_handle(mesh.prev_halfedge_handle(next), mesh.opposite_halfedge_handle(cur));
	}
	else if (!curIsNew && nextIsNew)
	{
		mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(next), mesh.next_halfedge_handle(cur));
	}
	else
	{
		if (mesh.next_halfedge_handle(cur) == next)
		{
			return;
		}
		Mesh::HalfedgeHandle heh = mesh.opposite_halfedge_handle(cur);
		do
		{
			heh = mesh.opposite_halfedge_handle(mesh.prev_halfedge_handle(heh));
		} while (mesh.face_handle(heh).is_valid() && heh != next && heh != mesh.opposite_halfedge_handle(cur));

		if (heh == next || heh == mesh.opposite_halfedge_handle(cur))
		{
			std::cout << "fail to find an opening to relink an existing face" << std::endl;
			assert(0);
		}

		Mesh::HalfedgeHandle open = mesh.prev_halfedge_handle(heh);
		mesh.set_next_halfedge_handle(open, mesh.next_halfedge_handle(cur));
		mesh.set_next_halfedge_handle(mesh.prev_halfedge_handle(next), heh);

	}
	mesh.set_next_halfedge_handle(cur, next);

}

Mesh::HalfedgeHandle InteractiveViewerWidget::vertex_split(Mesh::VertexHandle v0, Mesh::VertexHandle v1, Mesh::VertexHandle vl, Mesh::VertexHandle vr)
{
	Mesh::HalfedgeHandle v1vl, vlv1, vrv1, v0v1;

	// build loop from halfedge v1->vl
	if (vl.is_valid())
	{
		v1vl = mesh.find_halfedge(v1, vl);
		assert(v1vl.is_valid());
		vlv1 = insert_loop(v1vl);
	}

	// build loop from halfedge vr->v1
	if (vr.is_valid())
	{
		vrv1 = mesh.find_halfedge(vr, v1);
		assert(vrv1.is_valid());
		insert_loop(vrv1);
	}

	// handle boundary cases
	if (!vl.is_valid())
		vlv1 = mesh.prev_halfedge_handle(mesh.halfedge_handle(v1));
	if (!vr.is_valid())
		vrv1 = mesh.prev_halfedge_handle(mesh.halfedge_handle(v1));


	// split vertex v1 into edge v0v1
	v0v1 = insert_edge(v0, vlv1, vrv1);


	return v0v1;
}

Mesh::HalfedgeHandle InteractiveViewerWidget::insert_loop(Mesh::HalfedgeHandle _hh)
{
	Mesh::HalfedgeHandle  h0(_hh);
	Mesh::HalfedgeHandle  o0(mesh.opposite_halfedge_handle(h0));

	Mesh::VertexHandle    v0(mesh.to_vertex_handle(o0));
	Mesh::VertexHandle    v1(mesh.to_vertex_handle(h0));

	Mesh::HalfedgeHandle  h1 = mesh.new_edge(v1, v0);
	Mesh::HalfedgeHandle  o1 = mesh.opposite_halfedge_handle(h1);

	Mesh::FaceHandle      f0 = mesh.face_handle(h0);
	Mesh::FaceHandle      f1 = mesh.new_face();

	// halfedge -> halfedge
	mesh.set_next_halfedge_handle(mesh.prev_halfedge_handle(h0), o1);
	mesh.set_next_halfedge_handle(o1, mesh.next_halfedge_handle(h0));
	mesh.set_next_halfedge_handle(h1, h0);
	mesh.set_next_halfedge_handle(h0, h1);

	// halfedge -> face
	mesh.set_face_handle(o1, f0);
	mesh.set_face_handle(h0, f1);
	mesh.set_face_handle(h1, f1);

	// face -> halfedge
	mesh.set_halfedge_handle(f1, h0);
	if (f0.is_valid())
		mesh.set_halfedge_handle(f0, o1);


	// vertex -> halfedge
	mesh.adjust_outgoing_halfedge(v0);
	mesh.adjust_outgoing_halfedge(v1);
	return h1;
}

Mesh::HalfedgeHandle InteractiveViewerWidget::insert_edge(Mesh::VertexHandle _vh, Mesh::HalfedgeHandle _h0, Mesh::HalfedgeHandle _h1)
{
	assert(_h0.is_valid() && _h1.is_valid());

	Mesh::VertexHandle  v0 = _vh;
	Mesh::VertexHandle  v1 = mesh.to_vertex_handle(_h0);

	assert(v1 == mesh.to_vertex_handle(_h1));

	Mesh::HalfedgeHandle v0v1 = mesh.new_edge(v0, v1);
	Mesh::HalfedgeHandle v1v0 = mesh.opposite_halfedge_handle(v0v1);



	// vertex -> halfedge
	mesh.set_halfedge_handle(v0, v0v1);
	mesh.set_halfedge_handle(v1, v1v0);


	// halfedge -> halfedge
	mesh.set_next_halfedge_handle(v0v1, mesh.next_halfedge_handle(_h0));
	mesh.set_next_halfedge_handle(_h0, v0v1);
	mesh.set_next_halfedge_handle(v1v0, mesh.next_halfedge_handle(_h1));
	mesh.set_next_halfedge_handle(_h1, v1v0);


	// halfedge -> vertex
	for (Mesh::VertexIHalfedgeIter vih_it(mesh.vih_iter(v0)); vih_it.is_valid(); ++vih_it)
		mesh.set_vertex_handle(*vih_it, v0);


	// halfedge -> face
	mesh.set_face_handle(v0v1, mesh.face_handle(_h0));
	mesh.set_face_handle(v1v0, mesh.face_handle(_h1));


	// face -> halfedge
	if (mesh.face_handle(v0v1).is_valid())
		mesh.set_halfedge_handle(mesh.face_handle(v0v1), v0v1);
	if (mesh.face_handle(v1v0).is_valid())
		mesh.set_halfedge_handle(mesh.face_handle(v1v0), v1v0);


	// vertex -> halfedge
	mesh.adjust_outgoing_halfedge(v0);
	mesh.adjust_outgoing_halfedge(v1);


	return v0v1;
}

bool InteractiveViewerWidget::is_merge_ok(Mesh::EdgeHandle _eh)
{
	Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(_eh, 0);
	Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(_eh, 1);

	Mesh::VertexHandle v0 = mesh.from_vertex_handle(h0);
	Mesh::VertexHandle v1 = mesh.to_vertex_handle(h0);

	Mesh::FaceHandle f0 = mesh.face_handle(h0);
	Mesh::FaceHandle f1 = mesh.face_handle(h1);

	if (!f0.is_valid() || !f1.is_valid())
	{
		//_eh的一侧面不存在，但存在三角形结构，折叠e会减少一个洞
		Mesh::VertexHandle vh = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.next_halfedge_handle(h0)));
		if (vh == v0 && !f0.is_valid())
		{
			return false;
		}
		
		vh = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.next_halfedge_handle(h1)));
		if (vh == v1 && !f1.is_valid())
		{
			return false;
		}

	}
	else if (mesh.is_boundary(v0) && mesh.is_boundary(v1)) //_eh两侧有面
	{
		//_eh两端点存在半边在边界上
		return false;
	}
	else
	{
		//两个端点的公共邻接点与两端点不构成三角形
		Mesh::VertexHandle top, bottom;
		for (auto it = mesh.fv_begin(f0); it != mesh.fv_end(f0); it++)
		{
			auto vertex = it.handle();
			if (vertex != v0 && vertex != v1)
			{
				top = vertex;
				break;
			}
		}

		for (auto it = mesh.fv_begin(f1); it != mesh.fv_end(f1); it++)
		{
			auto vertex = it.handle();
			if (vertex != v0 && vertex != v1)
			{
				bottom = vertex;
				break;
			}
		}

		for (auto it0 = mesh.vv_begin(v0); it0 != mesh.vv_end(v0); it0++)
		{
			for (auto it1 = mesh.vv_begin(v1); it1 != mesh.vv_end(v1); it1++)
			{
				auto ver1 = it0.handle();
				auto ver2 = it1.handle();
				if (ver1 == ver2 && ver1 != top && ver1 != bottom)
				{
					return false;
				}
			}
		}

		//四面体为最简体，不能再被折叠
		//从_eh的任一端点出发的所有半边的下一条半边的反向半边组成一个三角形   说明_eh所在的部分为四面体
		std::vector<Mesh::HalfedgeHandle> tmp;
		Mesh::HalfedgeHandle he = mesh.halfedge_handle(v0);
		tmp.push_back(mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(he))));
		//while (mesh.to_vertex_handle(he) != v0)
		//{
		//	he = mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(he));
		//	tmp.push_back(mesh.opposite_halfedge_handle(he));
		//}

		for (Mesh::VertexOHalfedgeIter voh_it(mesh.voh_iter(v0)); voh_it.is_valid(); voh_it++)
		{
			tmp.push_back(mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(*voh_it)));
		}

		if (tmp.size() < 3)
		{
			return false;
		}

		if ((mesh.face_handle(tmp[0]) == mesh.face_handle(tmp[1])) && (mesh.face_handle(tmp[1]) == mesh.face_handle(tmp[2])))
		{
			return false;
		}
	}
	
	return true;
}

void InteractiveViewerWidget::mergeHalfedge(Mesh::HalfedgeHandle _heh)
{
	Mesh::HalfedgeHandle heh = _heh;
	if (mesh.is_boundary(heh))
	{	
		//半边为边界，只需要连接周边的半边
		mesh.set_halfedge_handle(mesh.to_vertex_handle(heh), mesh.next_halfedge_handle(heh));
		mesh.set_next_halfedge_handle(mesh.prev_halfedge_handle(heh), mesh.next_halfedge_handle(heh));
	}
	else
	{
		//半边不为边界，需要进行相连的面删除
		Mesh::EdgeHandle remove = mesh.edge_handle(mesh.next_halfedge_handle(heh));
		Mesh::EdgeHandle remian = mesh.edge_handle(mesh.prev_halfedge_handle(heh));
		Mesh::HalfedgeHandle outerLeft = mesh.opposite_halfedge_handle(mesh.prev_halfedge_handle(heh));
		Mesh::HalfedgeHandle outerRight = mesh.opposite_halfedge_handle(mesh.next_halfedge_handle(heh));

		mesh.set_next_halfedge_handle(mesh.prev_halfedge_handle(heh), mesh.next_halfedge_handle(outerRight));
		mesh.set_next_halfedge_handle(mesh.prev_halfedge_handle(outerRight), mesh.prev_halfedge_handle(heh));

		mesh.set_halfedge_handle(mesh.to_vertex_handle(heh), outerLeft);
		//mesh.adjust_outgoing_halfedge(mesh.to_vertex_handle(heh));
		//adjust_outgoing_halfedge(mesh.to_vertex_handle(heh));
		Mesh::VertexHandle top = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
		mesh.set_halfedge_handle(top, mesh.prev_halfedge_handle(heh));
		//mesh.adjust_outgoing_halfedge(top);
		Mesh::FaceHandle f = mesh.face_handle(outerRight);
		if (f.is_valid())
		{
			mesh.set_face_handle(mesh.prev_halfedge_handle(heh), f);
			if (mesh.halfedge_handle(f) == outerRight)
			{
				mesh.set_halfedge_handle(f, mesh.prev_halfedge_handle(heh));
			}
		}

		mesh.set_halfedge_handle(mesh.face_handle(heh), Mesh::InvalidHalfedgeHandle);
		mesh.status(mesh.face_handle(heh)).set_deleted(true);

		mesh.status(remove).set_deleted(true);
		if (mesh.has_halfedge_status())
		{
			mesh.status(outerRight).set_deleted(true);
			mesh.status(mesh.next_halfedge_handle(heh)).set_deleted(true);
		}

	}

	mesh.set_vertex_handle(heh, Mesh::InvalidVertexHandle);
	mesh.set_face_handle(heh, Mesh::InvalidFaceHandle);

	if (mesh.has_halfedge_status())
	{
		mesh.status(heh).set_deleted(true);
	}

	
}

void InteractiveViewerWidget::adjust_outgoing_halfedge(Mesh::VertexHandle _vh)
{
	for (Mesh::ConstVertexOHalfedgeIter vh_it = mesh.cvoh_iter(_vh); vh_it.is_valid(); ++vh_it)
	{
		if (mesh.is_boundary(*vh_it))
		{
			mesh.set_halfedge_handle(_vh, *vh_it);
			break;
		}
	}
}
