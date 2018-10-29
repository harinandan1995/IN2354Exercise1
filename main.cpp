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

bool isVectorValid(Vector4f vec) {

	return (vec.x() != MINF && vec.y() != MINF && vec.z() != MINF);

}

float distance(Vector4f a, Vector4f b) {

	return sqrtf(pow(a.x() - b.x(), 2) + pow(a.y() - b.y(), 2) + pow(a.z() - b.z(), 2));

}

bool canBeAFace(Vector4f a, Vector4f b, Vector4f c, float edgeThreshold) {

	if (isVectorValid(a) && isVectorValid(b) && isVectorValid(c)) {

		float ab = distance(a, b);
		float bc = distance(b, c);
		float ca = distance(c, a);

		if (ab > edgeThreshold || bc > edgeThreshold || ca > edgeThreshold) {
			
			return false;

		}
		else {

			return true;

		}

	}
	else {

		return false;

	}

}

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
	unsigned nFaces = width * height /2;

	// Write off file
	std::ofstream outFile(filename);
	if (!outFile.is_open()) return false;

	// write header
	outFile << "COFF" << std::endl;
	outFile << nVertices << " " << nFaces << " 0" << std::endl;

	// TODO: save vertices

	for (int idx = 0; idx < nVertices; idx++) {

		float positionX = vertices[idx].position.x();
		float positionY = vertices[idx].position.y();
		float positionZ = vertices[idx].position.z();

		if (positionX == MINF || positionY == MINF || positionZ == MINF) {

			outFile << "0 0 0 0 0 0" << std::endl;

		}
		else {

			outFile << positionX << " " << positionY << " " << positionZ << " 255 255 255 255" << std::endl;

		}

	}

	// TODO: save faces

	int totalFaces = 0;

	for (int i = 0; i < height; i++) {

		for (int j = 0; j < width; j++) {

			Vector4f current = vertices[i*width + j].position;

			if (current.x() != MINF && current.y() != MINF && current.z() != MINF) {

				if ( (j == 0 && i == height - 1) || (j == width -1 && i == 0)) continue;
				else if (i == 0) {

					Vector4f right = vertices[i*width + j + 1].position;
					Vector4f bottom = vertices[(i + 1)*width + j].position;

					if (canBeAFace(current, right, bottom, edgeThreshold)) {

						outFile << "3 " << i * width + j << " " << i * width + j + 1 << " " << (i + 1)*width + j << std::endl;
						totalFaces ++;
					}

				}
				else if (i == height - 1) {

					Vector4f left = vertices[i*width + j - 1].position;
					Vector4f top = vertices[(i - 1)*width + j].position;

					if (canBeAFace(current, left, top, edgeThreshold)) {

						outFile << "3 " << i * width + j << " " << i * width + j - 1 << " " << (i - 1)*width + j << std::endl;
						totalFaces++;
					}

				}
				else {

					Vector4f right = vertices[i*width + j + 1].position;
					Vector4f bottom = vertices[(i + 1)*width + j].position;
					Vector4f left = vertices[i*width + j - 1].position;
					Vector4f top = vertices[(i - 1)*width + j].position;
					if (canBeAFace(current, right, bottom, edgeThreshold)) {

						outFile << "3 " << i * width + j << " " << i * width + j + 1 << " " << (i + 1)*width + j << std::endl;
						totalFaces++;
					}
					if (canBeAFace(current, left, top, edgeThreshold)) {

						outFile << "3 " << i * width + j << " " << i * width + j - 1 << " " << (i - 1)*width + j << std::endl;
						totalFaces++;
					}

				}

			}
			

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

		std::cout << "Width: " << width << " Height:" << height << std::endl;

		for (int i = 0; i < height; i++) {

			for (int j = 0; j < width; j++) {

				int idx = i * width + j;

				float depth = depthMap[idx];

				if (depth == MINF) {

					vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
					vertices[idx].color = Vector4uc(0, 0, 0, 0);

				}
				else {

					float x = ((float)(i) - cX) / fovX;
					float y = ((float)(j) - cY) / fovY;

					Vector4f pixelCoordinates = Vector4f(depth*x, depth*y, depth, 1);
					Vector4f trasnformedPixelCoordinates = trajectoryInv * pixelCoordinates;

					vertices[idx].position = trasnformedPixelCoordinates;

				}

			}

		}

		/*for (int idx = 0; idx < width * height; idx++) {

			float depth = depthMap[idx];

			// std::cout << " Depth at " << idx << " is " << depth << std::endl;

			if (depth == MINF) {

				vertices[idx].position = Vector4f(MINF, MINF, MINF, MINF);
				vertices[idx].color = Vector4uc(0, 0, 0, 0);

			}
			else {

				//Vector4f pixelCoordinates = Vector4f(depth * (int)(idx / width), depth* (idx % width), depth, 1);
				float x = ((int)(idx / width) - cX) / fovX;
				float y = ((int)(idx % width) - cY) / fovY;

				Vector4f pixelCoordinates = Vector4f(depth*x, depth*y, depth, 1);

				vertices[idx].position = trajectoryInv*pixelCoordinates;

				// BYTE pixelColor = colorMap[idx];
				
				// std::cout << pixelColor << std::endl;

				// vertices[idx].color = Vector4uc(, );

			}

		}*/

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
