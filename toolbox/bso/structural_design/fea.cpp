#ifndef SD_FEA_CPP
#define SD_FEA_CPP

#include <sstream>
#include <stdexcept>
#include <vector>
#include <algorithm>

namespace bso { namespace structural_design {
	
	void fea::simplicialLLT()
	{
		mLLTSolver.compute(mGSM); 
		if (mLLTSolver.info() != Eigen::Success)
		{
			std::stringstream errorMessage;
			errorMessage << "\nWhen solving an FEA system with SimplicialLLT,\n"
									 << "Could not decompose the GSM\n"
									 << "(bso/structural_design/fea.cpp)" << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		
		for (auto& lc : mLoadCases)
		{
			try
			{
				mDisplacements[lc] = mLLTSolver.solve(mLoads[lc]);
				if (mLLTSolver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed");
				}
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nWhen solving FEA system with SimplicialLLT for load case: " << lc << "\n"
										 << "received the following error:\n" << e.what() << "\n"
										 << "(bso/structural_design/fea.cpp)" << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
	} // simplicialLLT()
	
	void fea::simplicialLDLT()
	{
		mLDLTSolver.compute(mGSM);
		if (mLDLTSolver.info() != Eigen::Success)
		{
			std::stringstream errorMessage;
			errorMessage << "\nWhen solving an FEA system with SimplicialLDLT,\n"
									 << "Could not decompose the GSM\n"
									 << "(bso/structural_design/fea.cpp)" << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		for (auto& lc : mLoadCases)
		{
			try
			{
				mDisplacements[lc] = mLDLTSolver.solve(mLoads[lc]);
				if (mLDLTSolver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed");
				}
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nWhen solving FEA system with SimplicialLDLT for load case: " << lc << "\n"
										 << "received the following error:\n" << e.what() << "\n"
										 << "(bso/structural_design/fea.cpp)" << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
	} // simplicialLDLT()
	
	void fea::BiCGSTAB()
	{
		Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::DiagonalPreconditioner<double>> solver;
		
		for (auto& lc : mLoadCases)
		{
			try
			{
				solver.compute(mGSM);
				solver.setTolerance(1e-3);
				
				if (solver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed decompose matrix GSM");
				}
				
				mDisplacements[lc] = solver.solve(mLoads[lc]);
				if (solver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed to solve GSM for loads");
				}
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nWhen solving FEA system with BiCGSTAB for load case: " << lc << "\n"
										 << "received the following error:\n" << e.what() << "\n"
										 << "(bso/structural_design/fea.cpp)" << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
	} // BiCGSTAB()
	
	void fea::scaledBiCGSTAB()
	{
		Eigen::BiCGSTAB<Eigen::SparseMatrix<double>, Eigen::DiagonalPreconditioner<double>> solver;
		
		for (auto& lc : mLoadCases)
		{
			try
			{
				solver.compute(mGSM);
				if (solver.info() != Eigen::Success)
				{
					throw std::runtime_error("solver could not decompose matrix GSM");
				}
				// get a rough solution
				solver.setMaxIterations(3);
				mDisplacements[lc] = solver.solve(mLoads[lc]);
				
				if (solver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed to roughly solve GSM for loads");
				}
				
				// scale the GSM into a temporary GSM matrix: Could
				Eigen::VectorXd wInverse = (mDisplacements[lc].array().abs()+1e-6).inverse();
				Eigen::SparseMatrix<double> C;
				C.resize(mDOFCount,mDOFCount);
				C = mGSM * wInverse.asDiagonal();
				
				solver.compute(C);
				solver.setMaxIterations(mDOFCount*10);
				
				if (solver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver could not decompose matrix C");
				}
				
				Eigen::VectorXd y(mDOFCount);
				y = solver.solve(mLoads[lc]);
				mDisplacements[lc] = wInverse.asDiagonal() * y;
				if (solver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed to solver for y");
				}
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nWhen solving FEA system with scaled BiCGSTAB for load case: " << lc << "\n"
										 << "received the following error:\n" << e.what() << "\n"
										 << "(bso/structural_design/fea.cpp)" << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
	} // scaledBiCGSTAB()
	
	fea::fea()
	{
		
	} // ctor
	
	fea::~fea()
	{
		for (auto& i : mNodes) delete i;
		for (auto& i : mElements) delete i;
	} // dtor
	
	element::node* fea::addNode(const bso::utilities::geometry::vertex& point)
	{
		for (auto i : mNodes)
		{
			if (i->isSameAs(point)) 
			{
				return i;
			}
		}
		unsigned int nodeID = mNodes.size()+1;
		mNodes.push_back(new element::node(point,nodeID));
		return mNodes.back();
	} // addNode()
	
	void fea::addElement(element::element* ele)
	{
		mElements.push_back(ele);
	} // addElement()
	
	void fea::generateGSM()
	{
		if (!mSystemInitialized)
		{
			mDOFCount = 0;
			for (auto& i : mNodes) i->generateNFT(mDOFCount);
			for (auto& i : mElements) i->generateEFT();
			
			std::cout << "generateNFT check" << std::endl;
			// get all the load cases
			for (auto& i : mNodes)
			{
				auto nodalLC = i->getLoadCases();
				for (auto& j : nodalLC)
				{
					if (std::find(mLoadCases.begin(), mLoadCases.end(),j) == mLoadCases.end())
					{
						mLoadCases.push_back(j);
					}
				}
			}
			
			// create and fill the load vectors
			for (auto& i : mLoadCases)
			{
				mLoads[i] = Eigen::VectorXd::Zero(mDOFCount);
				for (auto & j : mNodes)
				{
					Eigen::Vector6d nodalLoads;
					try
					{
						Eigen::Vector6d nodalLoads = j->getLoads(i);
					}
					catch (std::exception& e)
					{
						// this node does not have a load with this load case
						j->addLoadCase(i);
						Eigen::Vector6d nodalLoads = j->getLoads(i);
					}
					for (unsigned int k = 0; k < 6; ++k)
					{
						if (j->getNFS(k) == 0 || j->getConstraint(k) == 1) continue;
						unsigned int DOF = j->getGlobalDOF(k);
						mLoads[i](DOF) = nodalLoads(k);
					}
				}
			}
			
			// create the displacement vectors
			for (auto& i : mLoadCases) mDisplacements[i] = Eigen::VectorXd::Zero(mDOFCount);
			
			mSystemInitialized = true;
		}
	
		mGSM.resize(0,0); // clear it in case there are still any components left
		mGSM.resize(mDOFCount,mDOFCount); // size it to the numbe rof DOF''s in the system
		
		std::vector<element::triplet > triplets;
		for (const auto& i : mElements)
		{
			auto trips = i->getSMTriplets();
			triplets.insert(triplets.end(), trips.begin(), trips.end());
		}
		
		mGSM.setFromTriplets(triplets.begin(), triplets.end());
		tripletAddedValues = triplets;

	} // generateGSM()
	
	void fea::tieMethod(std::vector<element::node*> n, int r)
	{
		radius = r;
		originalDOF = mDOFCount; //Store the original amount of global DOFs
		cloudNodes = n;

		//Slave nodes are added to the master nodes
		for (auto& i : cloudNodes)
		{
			if (i->mMasterTieNode==true)
			{
				this->createCloud(i);
			}
			else{continue;}
		}

		//Check for all nodes that are not tied within the overlapping surfaces (set a larger radius)
		radius = sqrt(2*radius*radius);
		for (auto& i : clouds)
		{
			this->findSlaveNodes(i);
		}

		for (auto& i : cloudNodes)
		{
			if (i->inTieCloud!=true)
			{
				i->mMasterTieNode = false;
				i->mSlaveTieNode = false;
			}
		}
	}
	
	void fea::createCloud(element::node* i)
	{
		double iX = i->getX();
		double iY = i->getY();
		double iZ = i->getZ();
		
		for (auto& j : cloudNodes)
		{
			if (i == j){continue;}
			if (j->mSlaveTieNode!=true){continue;}
			if (j->inTieCloud==true){continue;}
			else
			{
				double jX = j->getX();
				double jY = j->getY();
				double jZ = j->getZ();
				
				double distance = 0;
				distance = (sqrt(((jX - iX)*(jX - iX)) + ((jY - iY)*(jY - iY)) + ((jZ - iZ)*(jZ - iZ))));
				
				if (distance < radius)
				{	
					//Creating a tie cloud
					unsigned long ID = clouds.size();
					clouds.push_back(new element::cloud(ID,*i));
					clouds[clouds.size()-1]->masterNode = i;
					i->inTieCloud = true;
					//std::cout << "Cloud created" << std::endl;
					
					this->findSlaveNodes(clouds[clouds.size()-1]);	
				}
				else {}
			}
		}
	}
	
	void fea::findSlaveNodes(element::cloud* i)
	{
		element::node* nodePtr;
		nodePtr = i->masterNode;
		
		double iX = nodePtr->getX();
		double iY = nodePtr->getY();
		double iZ = nodePtr->getZ();
		
		for (auto& j : cloudNodes)
		{
			if (nodePtr == j){continue;}
			else if (j->inTieCloud==true){continue;}
			else if (j->mMasterTieNode == true){continue;}
			else if (j->mSlaveTieNode == true)
			{
				double jX = j->getX();
				double jY = j->getY();
				double jZ = j->getZ();
				
				double distance = 0;
				distance = (sqrt(((jX - iX)*(jX - iX)) + ((jY - iY)*(jY - iY)) + ((jZ - iZ)*(jZ - iZ))));
				
				if (distance < radius)
				{	
					//Adding slave nodes
					i->slaveNodes.push_back(j);
					j->inTieCloud = true;
				}
			}
			else {}
		}
	}
	
	void fea::reduceStiffness()
	{
		for (auto& i : mElements)
		{
			if (i->isFlatShell())
			{
				int slaveNodes=0;
				for (auto& j : i->getNodes())
				{
					if(j->mSlaveTieNode)
					{
						slaveNodes++;
					}
					if(slaveNodes>2)//if element has more than 2 slave nodes then the stiffness will be reduced
					{
						//reduce stiffness of the slave elements
						i->reduceSlaveSM();
						break;
					}
				}
			}
			else{}continue;
		}
	}

	void fea::rebuildmGSM()
	{	
		clock_t tStart = clock();
		
		/*for(auto& i : clouds)
		{
			std::cout << "This cloud has master node: " << *i->masterNode << std::endl;
			std::cout << "With slave nodes:" << std::endl;
			for(auto& j : i->slaveNodes)
			{
				std::cout << *j << std::endl;
			}
		}*/

		std::cout << "Amount of clouds: " << clouds.size() << std::endl;

		//store all master and slave node DOFs
		for (auto& i : clouds)
		{
			for (auto& j : i->slaveNodes)
			{
				for(unsigned int k = 0; k < 6; ++k)
				{
					if(j->getNFS(k) == 0 || j->getConstraint(k) == 1) continue;
					slaveDOFs.push_back(j->getGlobalDOF(k));
				}
			}
		}
		copymGSM.setZero();//empty the copy of the GSM
		copymGSM = Eigen::MatrixXd(mGSM);//copy the GSM

		std::cout << "Values of rows will be added up" << std::endl;
		//adding up values from the slave rows to the master rows
		for (auto& i : clouds)
		{
			element::node* masterNode;
			masterNode = i->masterNode;
			for(unsigned int k = 0; k < 6; ++k)
			{
				if(masterNode->getNFS(k) == 0 || masterNode->getConstraint(k) == 1) continue;
				masterDOF = masterNode->getGlobalDOF(k);
				for (unsigned long c = 0; c < copymGSM.cols(); ++c)
				{
					for (auto& j : i->slaveNodes)
					{
						if(copymGSM(j->getGlobalDOF(k),c)==0)continue;
						copymGSM(masterDOF,c) += copymGSM(j->getGlobalDOF(k),c);
					}
				}
			}
		}

		std::cout << "Values of columns will be added up" << std::endl;
		//adding up values from the slave columns to the master columns
		for (auto& i : clouds)
		{
			element::node* masterNode;
			masterNode = i->masterNode;
			for(unsigned int k = 0; k < 6; ++k)
			{
				if(masterNode->getNFS(k) == 0 || masterNode->getConstraint(k) == 1) continue;
				masterDOF = masterNode->getGlobalDOF(k);
				for (unsigned long r = 0; r < copymGSM.cols(); ++r)
				{
					for (auto& j : i->slaveNodes)
					{
						if(copymGSM(r,j->getGlobalDOF(k))==0)continue;
						copymGSM(r,masterDOF) += copymGSM(r,j->getGlobalDOF(k));
					}
				}
			}
		}

		std::cout << "Wind load is removed" << std::endl;
		//removing windload on tied master nodes
		for (auto& i : mNodes)
		{
			if(!i->mMasterTieNode)continue;
			for(unsigned int k = 0; k < 6; ++k)
			{
				if(i->getNFS(k) == 0 || i->getConstraint(k) == 1) continue;
				int DOF;
				DOF = i->getGlobalDOF(k);
				if(k==0 || k == 1)
				{
					for (auto& i : mLoadCases)
					{
						mLoads[i](DOF)=0;
					}
				}
			}
		}

		std::cout << "Recount global DOFs" << std::endl;
		//adjusting the global degree of freedom of the tied nodes
		unsigned long tempDOFCount = 0;
		for (auto& i : mNodes)
		{
			if (i->mSlaveTieNode == true){continue;}
			i->generateNFT(tempDOFCount);
		}
		for (auto& i : clouds)
		{
			element::node* masterNode;
			masterNode = i->masterNode;
			for (unsigned int k = 0; k < 6; ++k)
			{
				if (masterNode->getNFS(k) == 0 || masterNode->getConstraint(k) == 1)
				{
					for (auto& j : i->slaveNodes)
					{
						if (j->getNFS(k) == 0 || j->getConstraint(k) == 1)continue;
						else{--mDOFCount;}
					}
					continue;
				}
				masterDOF = masterNode->getGlobalDOF(k);
				for (auto& j : i->slaveNodes)
				{
					j->updateNFT(k, masterDOF);
					--mDOFCount;
				}
			}
		}

		std::sort(slaveDOFs.begin(), slaveDOFs.end());
		std::vector<triplet> tripletmGSM;
		tripletmGSM.clear();

		std::cout << "Before removing rows and columns." << std::endl;
		//removing the slave rows and columns of the GSM
		long rowDOFindicator = 0;
		long rowValue = 0;
		for (unsigned long r = 0; r < mDOFCount; ++r, rowValue++)
		{
			long colValue = 0;
			long colDOFindicator = 0;
			while(slaveDOFs[rowDOFindicator] == rowValue && rowDOFindicator < slaveDOFs.size()){rowValue++,rowDOFindicator++;}
			for (unsigned long c = 0; c < mDOFCount; ++c, colValue++)
			{
				while(slaveDOFs[colDOFindicator] == colValue && colDOFindicator < slaveDOFs.size()){colValue++,colDOFindicator++;}
				if(copymGSM(rowValue,colValue)==0)continue;
				tripletmGSM.push_back(triplet(r,c,copymGSM(rowValue,colValue)));
			}
		}

		std::cout << "After removing rows and columns." << std::endl;
		mGSM.resize(0,0);
		mGSM.resize(mDOFCount,mDOFCount);	
		mGSM.setFromTriplets(tripletmGSM.begin(), tripletmGSM.end());


		//rebuild the load vector by deleteting the rows of the slave node
		Eigen::VectorXd tempLoads(mDOFCount);
		for (auto& i : mLoadCases)
		{
			rowValue = 0;
			long rowDOFindicator = 0;
			for (unsigned long r = 0; r < mDOFCount; ++r, rowValue++)
			{
				while(slaveDOFs[rowDOFindicator]==rowValue && rowDOFindicator < slaveDOFs.size()){++rowValue,++rowDOFindicator;}
				tempLoads(r) = mLoads[i](rowValue);
			}
			mLoads[i]=tempLoads;
		}
		//rebuild the displacement vector with the right amount of DOFs
		for (auto& i : mLoadCases) 
		{	
			mDisplacements[i] = Eigen::VectorXd::Zero(mDOFCount);
		}
		std::cout << "GSM is rebuild" << std::endl;
	}
	
	void fea::clearResponse()
	{
		for (auto& i : mElements) i->clearResponse();
		for (auto& i : mNodes) i->clearDisplacements();
		for (auto& i : mDisplacements) i.second.setZero();
	} // clearResponse()
	
	void fea::solve(std::string solver /*= "SimplicialLLT"*/)
	{
		msolver = solver;
		// solve the system with the specified solver 
		this->clearResponse();
		if (solver == "SimplicialLLT") this->simplicialLLT();
		else if (solver == "SimplicialLDLT") this->simplicialLDLT();
		else if (solver == "BiCGSTAB") this->BiCGSTAB();
		else if (solver == "scaledBiCGSTAB") this->scaledBiCGSTAB();
		else 
		{
			std::stringstream errorMessage;
			errorMessage << "\nTrying to solve FEA system with unknown solver:\n"
									 << solver << "\n"
									 << "(bso/structural_design/fea.cpp)" << std::endl;
			throw std::invalid_argument(errorMessage.str());
		}
		// add the displacements to the nodes
		for (auto& i : mNodes) i->addDisplacements(mDisplacements);
		// compute the responses for elements for every load case
		for (auto& i : mElements) 
		{
			for (auto& j : mLoadCases) 
			{
				i->computeResponse(j);
			}
		}		
	} // solve()

	Eigen::MatrixXd fea::solveAdjoint(Eigen::MatrixXd& ae) // for stress_based topopt
	{
		Eigen::MatrixXd Lambda;
		if (msolver == "SimplicialLLT")
		{
			try
			{
				Lambda = mLLTSolver.solve(ae);
				if (mLLTSolver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed");
				}
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nWhen solving an Adjoint system with SimplicialLLT \n"
										<< "received the following error:\n" << e.what() << "\n"
										<< "(bso/structural_design/fea.cpp)" << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
		else if (msolver == "SimplicialLDLT")
		{
			try
			{
				Lambda = mLDLTSolver.solve(ae);
				if (mLDLTSolver.info() != Eigen::Success)
				{
					throw std::runtime_error("Solver failed");
				}
			}
			catch (std::exception& e)
			{
				std::stringstream errorMessage;
				errorMessage << "\nWhen solving an Adjoint system with SimplicialLDLT \n"
										<< "received the following error:\n" << e.what() << "\n"
										<< "(bso/structural_design/fea.cpp)" << std::endl;
				throw std::runtime_error(errorMessage.str());
			}
		}
		else
		{
			std::stringstream errorMessage;
			errorMessage << "\nCould not solve Adjoint system with solver type: "
										<< msolver << std::endl;
			throw std::runtime_error(errorMessage.str());
		}
		return Lambda;
	} // solveAdjoint()

	bool fea::isSingular()
	{
		if (mGSM.nonZeros() == 0) return true;
		Eigen::JacobiSVD<Eigen::MatrixXd> SVD(mGSM);
		double cond = SVD.singularValues()(0) / SVD.singularValues()(SVD.singularValues().size()-1);
		return cond > 1e10;
	}

	Eigen::VectorXd fea::getDisplacements(element::load_case lc) const
	{
		auto dispSearch = mDisplacements.find(lc);
		if (dispSearch == mDisplacements.end())
		{
			std::stringstream errorMessage;
			errorMessage << "\nRequested displacements for unknown load case:\n"
									 << lc << "\n"
									 << "(bso/structural_design/fea.cpp)" << std::endl;
			throw std::invalid_argument(errorMessage.str());
		}
		return dispSearch->second;
	}
	
} // namespace structural_design
} // namespace bso

#endif // SD_FEA_CPP