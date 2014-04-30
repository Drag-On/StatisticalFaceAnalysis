//////////////////////////////////////////////////////////////////////
/// Statistical Face Analysis
///
/// Copyright (c) 2014 by Jan Moeller
///
/// This software is provided "as-is" and does not claim to be
/// complete or free of bugs in any way. It should work, but
/// it might also begin to hurt your kittens.
//////////////////////////////////////////////////////////////////////

#include "SFA/Utility/Model.h"

namespace sfa
{
    Model::Model()
    {
	// If no path is specified use a plane. This is just for testing
	// purposes where we don't want to load a file from HD.
	m_pMesh = dbgl::Mesh::makePlane(0);
	analyzeMesh();

	std::random_device rd;
	m_random.seed(rd());
    }

    Model::Model(std::string path)
    {
	dbgl::OBJMeshLoader loader;
	loader.setNormalCompatibilityAngle(std::numeric_limits<float>::max());
	m_pMesh = dbgl::Mesh::load(loader, path, 0 /* dbgl::Mesh::Optimize */);
	analyzeMesh();

	std::random_device rd;
	m_random.seed(rd());
    }

    Model::Model(Model const& other)
    {
	m_pMesh = new dbgl::Mesh(*other.m_pMesh);
	m_baseIndex2ModelIndex = other.m_baseIndex2ModelIndex;
	m_vertexTree = other.m_vertexTree;
	m_vertices = other.m_vertices;
	m_random = other.m_random;
    }

    Model::Model(Model&& other)
    {
	m_pMesh = other.m_pMesh;
	other.m_pMesh = nullptr;
	m_baseIndex2ModelIndex = other.m_baseIndex2ModelIndex;
	m_vertexTree = other.m_vertexTree;
	m_vertices = other.m_vertices;
	m_random = other.m_random;
    }

    Model& Model::operator=(Model const& other)
    {
	delete m_pMesh;
	m_pMesh = new dbgl::Mesh(*other.m_pMesh);
	m_baseIndex2ModelIndex = other.m_baseIndex2ModelIndex;
	m_vertexTree = other.m_vertexTree;
	m_vertices = other.m_vertices;
	m_random = other.m_random;
	return *this;
    }

    Model& Model::operator=(Model&& other)
    {
	if (this != &other)
	{
	    delete m_pMesh;
	    m_pMesh = other.m_pMesh;
	    other.m_pMesh = nullptr;
	    m_baseIndex2ModelIndex = other.m_baseIndex2ModelIndex;
	    m_vertexTree = other.m_vertexTree;
	    m_vertices = other.m_vertices;
	    m_random = other.m_random;
	}
	return *this;
    }

    Model::~Model()
    {
	delete m_pMesh;
    }

    unsigned int Model::getID() const
    {
	static unsigned int curMaxId = 0;
	return curMaxId++;
    }

    Vertex Model::getVertex(unsigned int n) const
    {
	if (n >= getAmountOfVertices())
	{
	    std::stringstream msg;
	    msg << "Vertex number out of bounds: " << n;
	    throw std::out_of_range(msg.str());
	}
	return m_vertices[n];
    }

    std::vector<Vertex> const& Model::getVertices() const
    {
	return m_vertices;
    }

    dbgl::KdTree<unsigned int, dbgl::Vec3d> const& Model::getVertexTree() const
    {
	return m_vertexTree;
    }

    void Model::setVertex(unsigned int n, Eigen::Vector3d const& coords, Eigen::Vector3d const& normal)
    {
	if (n >= getAmountOfVertices())
	{
	    std::stringstream msg;
	    msg << "Vertex number out of bounds: " << n;
	    throw std::out_of_range(msg.str());
	}

	// Update model data structure
	auto vertex = getVertex(n);

	// Get normal rotation
	auto rot = Eigen::Quaterniond::FromTwoVectors(vertex.normal, normal);

	// Store in own data structure
	vertex.coords = coords;
	vertex.normal = normal;
	m_vertices[n] = vertex;

	// Pass to base mesh
	for(auto i : vertex.baseVertices)
	{
	    m_pMesh->vertices()[i].x() = coords.x();
	    m_pMesh->vertices()[i].y() = coords.y();
	    m_pMesh->vertices()[i].z() = coords.z();
	    Eigen::Vector3d oldNormal(m_pMesh->normals()[i].x(), m_pMesh->normals()[i].y(), m_pMesh->normals()[i].z());
	    auto newNormal = rot * oldNormal;
	    m_pMesh->normals()[i].x() = newNormal[0];
	    m_pMesh->normals()[i].y() = newNormal[1];
	    m_pMesh->normals()[i].z() = newNormal[2];
	}
    }

    unsigned int Model::getAmountOfVertices() const
    {
	return m_vertices.size();
    }

    Eigen::Vector3d Model::getAverage() const
    {
	Eigen::Vector3d average(0, 0, 0);
	unsigned int amount = getAmountOfVertices();
	for (unsigned int i = 0; i < amount; i++)
	{
	    average += getVertex(i).coords;
	}
	average /= amount;
	return average;
    }

    void Model::addNoise()
    {
	// Initialize random number generator
	std::uniform_real_distribution<float> rand_float(-0.05f, 0.05f);
	// Iterate all vertices and translate them randomly along their normal
	for(unsigned int i = 0; i < getAmountOfVertices(); i++)
	{
	    auto vertex = getVertex(i);
	    auto coeff = rand_float(m_random);
	    auto newCoords = vertex.coords + coeff * vertex.normal;
	    setVertex(vertex.id, newCoords, vertex.normal);
	}
    }

    void Model::addHole()
    {
	// Initialize random number generator
	std::uniform_int_distribution<uint32_t> rand_uint_0_vertices(0, getAmountOfVertices() - 1);
	// Generate index of vertex to remove
	auto index = rand_uint_0_vertices(m_random);
	// Note: we need to iterate from high indices to low indices since every removed vertex will invalidate
	// every other vertex with an index higher than their own index. Indices are then regenerated in analyzeMesh().
	for(auto it = m_vertices[index].baseVertices.rbegin(); it != m_vertices[index].baseVertices.rend(); ++it)
	    m_pMesh->removeVertex(*it);
	analyzeMesh();
    }

    void Model::rotateRandom(double maxAngle)
    {
	std::uniform_real_distribution<double> rand_double_nMax_max(-maxAngle, maxAngle);
	double angle = rand_double_nMax_max(m_random);
	Eigen::Vector3d axis = Eigen::Vector3d::Random();
	axis.normalize();
	Eigen::AngleAxis<double> aa(angle, axis);
	for(unsigned int i = 0; i < m_pMesh->vertices().size(); i++)
	{
	    auto vert = m_pMesh->getVertices()[i];
	    Eigen::Vector3d coords(vert.x(), vert.y(), vert.z());
	    coords = aa.toRotationMatrix() * coords;
	    m_pMesh->vertices()[i] = dbgl::Vec3f(coords[0], coords[1], coords[2]);
	}
	analyzeMesh();
    }

    void Model::translateRandom(double maxTranslation)
    {
	Eigen::Vector3d translation = Eigen::Vector3d::Random();
	translation.normalize();
	translation *= maxTranslation;
	for(unsigned int i = 0; i < m_pMesh->vertices().size(); i++)
	    m_pMesh->vertices()[i].translate(translation[0], translation[1], translation[2]);
	analyzeMesh();
    }

    dbgl::Mesh* Model::getBasePointer()
    {
	return m_pMesh;
    }

    void Model::analyzeMesh()
    {
	// Clear any previous results
	m_vertices.clear();
	m_vertexTree.clear();
	m_baseIndex2ModelIndex.clear();

	// Add all vertices
	for (unsigned int i = 0; i < m_pMesh->getVertices().size(); i++)
	{
	    auto vertex = m_pMesh->getVertices()[i];
	    auto normal = m_pMesh->getNormals()[i];

	    // Check if there already is a vertex with the same coordinates
	    auto dbglCoords = dbgl::Vec3d(vertex.x(), vertex.y(), vertex.z());
	    auto pVertId = m_vertexTree.getSimilar(dbglCoords, 0.0001);
	    if(pVertId != nullptr)
	    {
		// Average normals
		m_vertices[*pVertId].normal += Eigen::Vector3d(normal.x(), normal.y(), normal.z());
		m_vertices[*pVertId].normal.normalize();
		// Add base vertex id
		m_vertices[*pVertId].baseVertices.insert(i);
		// Create a list base index -> this index
		m_baseIndex2ModelIndex.push_back(*pVertId);
	    }
	    else
	    {
		Vertex vert;
		vert.id = m_vertices.size();
		vert.coords = Eigen::Vector3d(vertex.x(), vertex.y(), vertex.z());
		vert.normal = Eigen::Vector3d(normal.x(), normal.y(), normal.z());
		vert.baseVertices.insert(i);
		m_vertices.push_back(vert);
		m_vertexTree.insert(dbglCoords, vert.id);
		// Create a list base index -> this index
		m_baseIndex2ModelIndex.push_back(vert.id);
	    }
	}

	// Balance the tree for maximum performance
	m_vertexTree.balance();

	// Compute neighbors
	for (unsigned int i = 0; i < m_pMesh->getIndices().size(); i += 3)
	{
	    m_vertices[m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 0]]].neighbors.insert(
		    m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 1]]);
	    m_vertices[m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 0]]].neighbors.insert(
		    m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 2]]);
	    m_vertices[m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 1]]].neighbors.insert(
		    m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 0]]);
	    m_vertices[m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 1]]].neighbors.insert(
		    m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 2]]);
	    m_vertices[m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 2]]].neighbors.insert(
		    m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 0]]);
	    m_vertices[m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 2]]].neighbors.insert(
		    m_baseIndex2ModelIndex[m_pMesh->getIndices()[i + 1]]);
	}

	// Compute edge vertices
	for (unsigned int i = 0; i < m_vertices.size(); i++)
	{
	    if (m_vertices[i].neighbors.empty())
		continue;
	    bool isEdge = false;
	    for(auto it = m_vertices[i].neighbors.begin(); it != m_vertices[i].neighbors.end(); ++it)
	    {
		// Usually this should finish on first iteration. Only due to bad luck it might need more.
		auto start = m_vertices[*it];
		isEdge = checkEdge(m_vertices[i], start);
		if(!isEdge)
		    break;
	    }

	    m_vertices[i].isEdge = isEdge;
	}
    }

    bool Model::checkEdge(Vertex base, Vertex start)
    {
	std::map<unsigned int, bool> checked;
	for(auto neighbor : base.neighbors)
	    checked[neighbor] = false;
	checked[start.id] = true;
	return checkEdge(base, start, start, start, checked);
    }

    bool Model::checkEdge(Vertex base, Vertex start, Vertex begin, Vertex last,
	    std::map<unsigned int, bool> checked)
    {
	// Get a vertex that is both a neighbor of base and start and different than last
	for (auto it = start.neighbors.begin(); it != start.neighbors.end(); ++it)
	{
	    // If the neighbors neighbor is the beginning but not the one we just came from
	    // then there is a way of circling base through their neighbors
	    if(*it == begin.id && *it != last.id)
		return false;
	    // If the neighbor has already been used don't use it again
	    if(checked[*it])
		continue;
	    for (auto it2 = base.neighbors.begin(); it2 != base.neighbors.end(); ++it2)
	    {
		if (*it == *it2)
		{
		    // Found a match
		    checked[*it] = true;
		    // Check the next vertex
		    bool isEdge = checkEdge(base, m_vertices[*it], begin, start, checked);
		    // If checkEdge returns true there might still be a different "path" to prove
		    // that it's no edge, thus we continue iterating
		    if (!isEdge)
			return false;
		}
	    }
	}
	// If we get here the algorithm has not found a proper way
	return true;
    }
}
