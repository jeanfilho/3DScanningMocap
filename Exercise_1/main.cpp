#include <iostream>
#include <fstream>

#include "Eigen.h"

#include "VirtualSensor.h"

struct Vertex
{
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	
	// position stored as 4 floats (4th component is supposed to be 1.0)
	Vector4f position;
	// color stored as 4 unsigned char
	Vector4uc color;
};

bool WriteMesh(Vertex* vertices, unsigned int width, unsigned int height, const std::string& filename)
{
	float edgeThreshold = 0.01f; // 1cm

	// TODO 2: use the OFF file format to save the vertices grid (http://www.geomview.org/docs/html/OFF.html)
	// - have a look at the "off_sample.off" file to see how to store the vertices and triangles
	// - for debugging we recommend to first only write out the vertices (set the number of faces to zero)
	// - for simplicity write every vertex to file, even if it is not valid (position.x() == MINF) (note that all vertices in the off file have to be valid, thus, if a point is not valid write out a dummy point like (0,0,0))
	// - use a simple triangulation exploiting the grid structure (neighboring vertices build a triangle, two triangles per grid cell)
	// - you can use an arbitrary triangulation of the cells, but make sure that the triangles are consistently oriented
	// - only write triangles with valid vertices and an edge length smaller then edgeThreshold

	// TODO: Get number of vertices
	unsigned int nVertices = width * height;

	// TODO: Get number of faces
	unsigned nFaces = (width - 1) * (height - 1) * 2;




	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices
	for(int y = 0; y < height; y++)
	{
		for(int x = 0; x < width; x++)
		{
			Vector4f pos = vertices[y * width + x].position.x() != MINF ? vertices[y * width + x].position : Vector4f(0,0,0,0);
			Vector4uc color = vertices[y * width + x].color;
			outFile << pos[0] << " " << pos[1] << " " << pos[2] << " ";
			outFile << static_cast<int>(color[0]) << " " << static_cast<int>(color[1]) << " " << static_cast<int>(color[2]) << " " << static_cast<int>(color[3]) << std::endl;
		}
	}

	// TODO: save faces
	for (int y = 0; y < height - 1; y++)
	{
		for (int x = 0; x < width - 1; x++)
		{
			const int i = y * width + x;
			Vector4f v1 = vertices[i].position;
			Vector4f v2 = vertices[i + width].position;
			Vector4f v3 = vertices[i + 1].position;

			if (v1[0] != MINF && v2[0] != MINF && v3[0] != MINF
				&& (v2 - v1).norm() < edgeThreshold
				&& (v2 - v3).norm() < edgeThreshold
				&& (v3 - v1).norm() < edgeThreshold)
				outFile << 3 << " " << i << " " << i + width << " " << i + 1 << std::endl;

			v1 = vertices[i + width].position;
			v2 = vertices[i + 1 + width].position;
			v3 = vertices[i + 1].position;

			if (v1[0] != MINF && v2[0] != MINF && v3[0] != MINF
				&& (v2 - v1).norm() < edgeThreshold
				&& (v2 - v3).norm() < edgeThreshold
				&& (v3 - v1).norm() < edgeThreshold)
				outFile << 3 << " " << i + width << " " << i + 1 + width << " " << i + 1 << std::endl;
		}
	}





	// close file
	outFile.close();

	return true;
}

int main()
{
	std::string filenameIn = "./data/rgbd_dataset_freiburg1_xyz/";
	std::string filenameBaseOut = "mesh_";

	// load video
	std::cout << "Initialize virtual sensor..." << std::endl;
	VirtualSensor sensor;
	if (!sensor.Init(filenameIn))
	{
		std::cout << "Failed to initialize the sensor!\nCheck file path!" << std::endl;
		return -1;
	}

	// convert video to meshes
	while (sensor.ProcessNextFrame())
	{
		// get ptr to the current depth frame
		// depth is stored in row major (get dimensions via sensor.GetDepthImageWidth() / GetDepthImageHeight())
		float* depthMap = sensor.GetDepth();
		// get ptr to the current color frame
		// color is stored as RGBX in row major (4 byte values per pixel, get dimensions via sensor.GetColorImageWidth() / GetColorImageHeight())
		BYTE* colorMap = sensor.GetColorRGBX();

		// get depth intrinsics
		Matrix3f depthIntrinsics = sensor.GetDepthIntrinsics();
		float fovX = depthIntrinsics(0, 0);
		float fovY = depthIntrinsics(1, 1);
		float cX = depthIntrinsics(0, 2);
		float cY = depthIntrinsics(1, 2);

		// compute inverse depth extrinsics
		Matrix4f depthExtrinsicsInv = sensor.GetDepthExtrinsics().inverse();

		Matrix4f trajectory = sensor.GetTrajectory();
		Matrix4f trajectoryInv = sensor.GetTrajectory().inverse();

		// TODO 1: back-projection
		// write result to the vertices array below, keep pixel ordering!
		// if the depth value at idx is invalid (MINF) write the following values to the vertices array
		// vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
		// vertices[idx].color = Vector4uc(0,0,0,0);
		// otherwise apply back-projection and transform the vertex to world space, use the corresponding color from the colormap
		Vertex* vertices = new Vertex[sensor.GetDepthImageWidth() * sensor.GetDepthImageHeight()];

		int width = sensor.GetDepthImageWidth();
		int height = sensor.GetDepthImageHeight();

		for(int y = 0; y < height; y++)
		{
			for(int x = 0; x < width; x++)
			{
				const int index = y * width + x;
				Vector4f position;
				Vector4uc color;

				if(depthMap[index] == MINF)
				{
					position = Vector4f(MINF, MINF, MINF, MINF);
					color = Vector4uc(0, 0, 0, 0);
				}
				else
				{
					Vector3f temp = depthIntrinsics.inverse() * Vector3f(float(x), float(y), depthMap[index]);
					position = Vector4f(temp.x(), temp.y(), temp.z(), 1.0f);
					position = trajectoryInv * depthExtrinsicsInv * position;
					//std::cout << position << std::endl;
					color = Vector4uc(colorMap[index * 4], colorMap[index * 4 + 1], colorMap[index * 4 + 2], colorMap[index * 4 + 3]);					
				}
				
				vertices[index].position = position;
				vertices[index].color = color;
			}
		}




		// write mesh file
		std::stringstream ss;
		ss << filenameBaseOut << sensor.GetCurrentFrameCnt() << ".off";
		if (!WriteMesh(vertices, sensor.GetDepthImageWidth(), sensor.GetDepthImageHeight(), ss.str()))
		{
			std::cout << "Failed to write mesh!\nCheck file path!" << std::endl;
			return -1;
		}

		// free mem
		delete[] vertices;
	}

	return 0;
}
