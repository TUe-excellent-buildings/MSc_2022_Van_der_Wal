#ifndef CF_TRIPRISM_CPP
#define CF_TRIPRISM_CPP

namespace bso { namespace spatial_design { namespace conformal {
	
	cf_triPrism::cf_triPrism(const utilities::geometry::triangular_prism& rhs, cf_geometry_model* geometryModel)
	: utilities::geometry::triangular_prism(rhs, geometryModel->tolerance())
	{
		mGeometryModel = geometryModel;
		for (const auto& i : mVertices)
		{
			mCFVertices.push_back(mGeometryModel->addVertex(i));
			mCFVertices.back()->addTriPrism(this);
		}
		for (const auto& i : mLineSegments)
		{
			mCFLines.push_back(mGeometryModel->addLine(i));
			mCFLines.back()->addTriPrism(this);
		}
		for (const auto& i : mPolygons)
		{
			if((i->getVertices()).size() == 3)
			{
				mCFTriangles.push_back(mGeometryModel->addTriangle(*i));
				mCFTriangles.back()->addTriPrism(this);
			}
			else if((i->getVertices()).size() == 4)
			{
				mCFRectangles.push_back(mGeometryModel->addRectangle(*i));
				mCFRectangles.back()->addTriPrism(this);
			}
			else
			{
				std::stringstream errorMessage;
					errorMessage << "\nError, found a polygon with more then the allowed amound of vertices\n"
											 << "The amound of vertices should be either 3 or 4 and is " << mVertices.size() << "\n"
											 << "(bso/spatial_design/conformal/cf_triprism)." << std::endl;
					throw std::runtime_error(errorMessage.str());
			}
		}
	} // ctor
	
	cf_triPrism::~cf_triPrism()
	{ //function still needs to be updated
		for (const auto& i : mCFVertices) i->removeTriPrism(this);
		for (const auto& i : mCFLines) i->removeTriPrism(this);
		for (const auto& i : mCFTriangles) i->removeTriPrism(this);
		for (const auto& i : mCFRectangles) i->removeTriPrism(this);
	} // dtor
	
	
	/*
	void cf_triPrism::split(cf_vertex* pPtr) // not updated for this class, since this isn't needed
	{ // 
		for (auto& i : mCFVertices)
		{
			if (pPtr == i) return;
		}

		std::vector<cf_vertex*> newVertices;
		std::vector<cf_tetrahedron*> newTetrahedron;
		bool split = false;
		
		
		//if (this->isInside(*pPtr, mGeometryModel->tolerance()))
		//{
			
		//}
		
		
		//std::cout << "This tetrahedron is split: " << *this << std::endl;
		//std::cout << "in acordance to vetex: " << *pPtr << std::endl;
		
		if (!split)
		{
			for (unsigned int i = 0; i < 4; ++i)
			{ // check if the point is on any of the triangles of this tetrahedron
				if (mPolygons[i]->isInside(*pPtr, mGeometryModel->tolerance()))
				{
				 // if it is, lets split the tetrahedron into three new ones
					
					split = true;
					newVertices.push_back(pPtr);
					unsigned int opposite = 4;
					
					// first find the vertex opposite to triangle i
					for (unsigned int j = 0; j < 4; ++j)
					{
						if ((std::find((mPolygons[i]->getVertices()).begin(),(mPolygons[i]->getVertices()).end(), mVertices[j]) == (mPolygons[i]->getVertices()).end()))
						{
							opposite = j;
							break;
						}
					}
					if (opposite == 4)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, could not find opposite vertex when\n"
												 << "splitting a tetrahedron from a point on a triangle\n"
												 << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					// generate new tetrahedrons
					for (const auto k: mPolygons[i]->getLines())
					{
						std::vector<utilities::geometry::vertex> cornerPoints;
						cornerPoints.push_back(*pPtr);
						cornerPoints.push_back(mVertices[opposite]);
						cornerPoints.push_back(k[0]);
						cornerPoints.push_back(k[1]);
						
						newVertices.push_back(mGeometryModel->addVertex(k[0]));
						newVertices.push_back(mGeometryModel->addVertex(k[1]));
						
						if (cornerPoints.size() != 4)
						{
							std::stringstream errorMessage;
							errorMessage << "\nError, expected to find 4 new vertices,\n"
													 << "when dividing a tetrahedron at a triangle intersection.\n"
													 << "Found: " << newVertices.size() << "\n"
													 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
							throw std::runtime_error(errorMessage.str());
						}
						for(int l = 0; l < cornerPoints.size(); l++)
						{
							for(int m=0; m < cornerPoints.size(); m++)
							{
								if(l == m)
								{
									continue;
								}
								else
								{
									if(cornerPoints[l].isSameAs(cornerPoints[m], mGeometryModel->tolerance()))
									{
										std::stringstream errorMessage;
										errorMessage << "\nError, expected to find 4 unique vertices,\n"
																 << "when dividing a tetrahedron at a triangle intersection.\n"
																 << "Found: " << newVertices.size() << "\n"
																 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
							}
						}
					
					
						newTetrahedron.push_back(mGeometryModel->addTetrahedron(
							utilities::geometry::tetrahedron(cornerPoints, mGeometryModel->tolerance())));
					}
					
					if (newTetrahedron.size() != 3)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, expected to find 2 new tetrahedrons,\n"
												 << "when dividing a tetrahedron at a triangle segment.\n"
												 << "Found: " << newTetrahedron.size() << "\n"
												 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					break;
				}
			}
		}
		
		
		if (!split)
		{
			for (unsigned int i = 0; i < 6; ++i)
			{
				if (mLineSegments[i].isOnLine(*pPtr,mGeometryModel->tolerance()))
				{
					split = true;
					
					newVertices.push_back(pPtr);
					std::vector<int> opposite;
					
					// find the 2 vertexes not conected to the intersected line
					for (unsigned int j = 0; j < 4; ++j)
					{
						if (mLineSegments[i][0] != mVertices[j])
						{
							if (mLineSegments[i][1] != mVertices[j])
							{
								opposite.push_back(j);
							}
						}
					}
					
					if (opposite.size() != 2)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, could not find 2 opposite vertexes to line_segment \n"
												 << " when splitting a tetrahedron from a point on a line\n"
												 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					
					
					// generate new tetrahedrons
					for (unsigned int j = 0; j < 2; ++j)
					{
						std::vector<utilities::geometry::vertex> cornerPoints;
						cornerPoints.push_back(*pPtr);
						cornerPoints.push_back((mLineSegments[i])[j]);
						newVertices.push_back(mGeometryModel->addVertex(((mLineSegments[i])[j])));
						for(const int k: opposite)
						{
							cornerPoints.push_back(mVertices[k]);
							newVertices.push_back(mGeometryModel->addVertex((mVertices[k])));
						}
						
						if (cornerPoints.size() != 4)
						{
							std::stringstream errorMessage;
							errorMessage << "\nError, expected to find 4 new vertices,\n"
													 << "when dividing a tetrahedron at a line segment.\n"
													 << "Found: " << newVertices.size() << "\n"
													 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
							throw std::runtime_error(errorMessage.str());
						}
						for(int l = 0; l < cornerPoints.size(); l++)
						{
							for(int m=0; m < cornerPoints.size(); m++)
							{
								if(l == m)
								{
									continue;
								}
								else
								{
									if(cornerPoints[l].isSameAs(cornerPoints[m], mGeometryModel->tolerance()))
									{
										std::stringstream errorMessage;
										errorMessage << "\nError, expected to find 4 unique vertices,\n"
																 << "when dividing a tetrahedron at a line segment.\n"
																 << "Found: " << newVertices.size() << "\n"
																 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
										throw std::runtime_error(errorMessage.str());
									}
								}
							}
						}
					
						newTetrahedron.push_back(mGeometryModel->addTetrahedron(
							utilities::geometry::tetrahedron(cornerPoints, mGeometryModel->tolerance())));
					}
					
					if (newTetrahedron.size() != 2)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, expected to find 2 new tetrahedrons,\n"
												 << "when dividing a tetrahedron at a line segment.\n"
												 << "Found: " << newTetrahedron.size() << "\n"
												 << "(bso/spatial_design/conformal/cf_tetrahedron)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					break;
				}
			}
		}
		
		if (split)
		{
			mDeletion = true;
			for (auto& i : mCFSpaces)
			{				
				i->removeTetrahedron(this);
				for (auto& j : newTetrahedron)
				{
					i->addTetrahedron(j);
					j->addSpace(i);
				}
			}
			
			this->checkAssociatedT(newTetrahedron);
			
			mCFSpaces.clear();
		}
	} // split
	*/

	
	/*
	void cf_triPrism::checkAssociatedT(std::vector<cf_triangular_prism*> newTriPrism) // not updated for this class, since this isn't needed
	{
		
		// insert the found triangles in the triangle split function to split the origonal triangle
		for (const auto& j : cfTriangles()) // origonal triangles
		{
			std::vector<cf_triangle*> newTri;
			
			for (const auto i : newTetrahedron)
			{
				for (const auto k : i->cfTriangles()) // new triangles
				{
					std::vector <bool> OnOrigonal;
					for (const auto l : k->getVertices()) // vertices of new triangle
					{
						bool found = false;
						for (const auto m : j->getVertices()) // new triangles
						{
							if(m.isSameAs(l, mGeometryModel->tolerance()))
							{
								OnOrigonal.push_back(true);
								found = true;
							}
						}
						if(j->isInsideOrOn(l, mGeometryModel->tolerance()) && found == false)
						{
							OnOrigonal.push_back(true);
						}
						else if(found == false)
						{
							OnOrigonal.push_back(false);
						}
					}

					bool isOnOgigonal = true;
					
					for(const auto o: OnOrigonal)
					{
						if(o == false)
						{
							isOnOgigonal = false;
							break;
						}
					}
					
					if(isOnOgigonal == false)
					{
						continue;
					}					
					else if(isOnOgigonal == true)
					{
						newTri.push_back(k);
					}
				}
			}
			
			if(newTri.size() > 2)
			{
				j -> splitT(newTri);
			}
		}
		
		// insert the found line segments in the line split function to split the origonal lines
		for (const auto& j : cfLines()) // origonal line segments
		{
			std::vector<cf_line*> newLine;			
			
			for (auto i : newTetrahedron)
			{
				std::vector <bool> origonalLinecheck;
							
				for (auto m : (i->cfLines())) // lines of the new tetrahedron
				{
					for (auto n = m->begin(); n != m->end(); n++)
					{
						if(j ->isOnLine(*n, mGeometryModel->tolerance()))
						{
							origonalLinecheck.push_back(true);
						}
						else
						{
							for (auto k = j->begin(); k != j->end(); k++)
							{
								if(k->isSameAs(*n, mGeometryModel->tolerance()))
								{
									origonalLinecheck.push_back(true);
								}
							}
						}
					}
					
					if(origonalLinecheck.size() == 2) // all vertixes of new polygons are on the orgigonal polygon
					{
						newLine.push_back(m);
					}
					else if (origonalLinecheck.size() > 2)
					{
						std::stringstream errorMessage;
						errorMessage << "\nError, something went wrong when searching for the new line within the old one,\n"
												  << "(bso/spatial_design/conformal/cf_cuboid)." << std::endl;
						throw std::runtime_error(errorMessage.str());
					}
					origonalLinecheck.clear();
				}	
			}
			
			
			// Insert new lines to line split function
			if(newLine.size() > 1)
			{
				j -> splitN(newLine);
			}
		}
	}//checkAssociatedT
	*/

} // conformal
} // spatial_design
} // bso

#endif // CF_TRIPRISM_CPP