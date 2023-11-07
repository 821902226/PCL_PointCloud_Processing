#include <vtkSTLReader.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCleanPolyData.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkExtractSelection.h>
#include <vtkFillHolesFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkInformation.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyDataNormals.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkUnstructuredGrid.h>
#include <vtkBYUReader.h>
#include <vtkOBJReader.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataReader.h>
#include <vtkSTLReader.h>
#include <vtkSphereSource.h>
#include <vtkXMLPolyDataReader.h>
#include <vtksys/SystemTools.hxx>
#include <vtkSTLWriter.h>
#include <vtkPLYWriter.h>

namespace {
	vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileName);
}

int main(int argc, char* argv[])
{
	vtkNew<vtkNamedColors> colors;

	//读取ply文件
	auto input = ReadPolyData("bunny_wnnm_tri.ply");


	vtkNew<vtkFillHolesFilter> fillHolesFilter;
	fillHolesFilter->SetInputData(input);
	fillHolesFilter->SetHoleSize(10.0);
	fillHolesFilter->Update();

	
	vtkNew<vtkPolyDataNormals> normals;
	normals->SetInputData(fillHolesFilter->GetOutput());
	normals->ConsistencyOn();
	normals->SplittingOff();
	normals->Update();

	std::string saveFileName = "bunny_wnnm_hole.ply";
	vtkSmartPointer<vtkPLYWriter> stlWriter =
		vtkSmartPointer<vtkPLYWriter>::New();
	stlWriter->SetFileName(saveFileName.c_str());
	stlWriter->SetInputConnection(normals->GetOutputPort());
	stlWriter->Write();
	std::cout << "stlWriter sucessful " << std::endl;

#if 0
	// Restore the original normals
	normals->GetOutput()->GetPointData()->
		SetNormals(input->GetPointData()->GetNormals());
#endif
	// 点云可视化
	// 定义可视化窗口大小
	// (xmin, ymin, xmax, ymax)
	double leftViewport[4] = { 0.0, 0.0, 0.5, 1.0 };
	double rightViewport[4] = { 0.5, 0.0, 1.0, 1.0 };

	
	vtkNew<vtkPolyDataMapper> originalMapper;
	originalMapper->SetInputData(input);

	vtkNew<vtkProperty> backfaceProp;
	backfaceProp->SetDiffuseColor(colors->GetColor3d("Banana").GetData());

	vtkNew<vtkActor> originalActor;
	originalActor->SetMapper(originalMapper);
	originalActor->SetBackfaceProperty(backfaceProp);
	originalActor->GetProperty()->SetDiffuseColor(
		colors->GetColor3d("NavajoWhite").GetData());

	vtkNew<vtkPolyDataMapper> filledMapper;
	filledMapper->SetInputData(fillHolesFilter->GetOutput());

	vtkNew<vtkActor> filledActor;
	filledActor->SetMapper(filledMapper);
	filledActor->GetProperty()->SetDiffuseColor(
		colors->GetColor3d("NavajoWhite").GetData());
	filledActor->SetBackfaceProperty(backfaceProp);

	// 创建渲染器、渲染窗口和交互器
	vtkNew<vtkRenderer> leftRenderer;
	leftRenderer->SetViewport(leftViewport);

	vtkNew<vtkRenderer> rightRenderer;
	rightRenderer->SetViewport(rightViewport);

	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->SetSize(600, 300);
	renderWindow->SetWindowName("FillHoles");

	renderWindow->AddRenderer(leftRenderer);
	renderWindow->AddRenderer(rightRenderer);

	vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
	renderWindowInteractor->SetRenderWindow(renderWindow);

	// 把模型添加进场景
	leftRenderer->AddActor(originalActor);
	rightRenderer->AddActor(filledActor);
	leftRenderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

	leftRenderer->GetActiveCamera()->SetPosition(0, -1, 0);
	leftRenderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
	leftRenderer->GetActiveCamera()->SetViewUp(0, 0, 1);
	leftRenderer->GetActiveCamera()->Azimuth(30);
	leftRenderer->GetActiveCamera()->Elevation(30);

	leftRenderer->ResetCamera();

	rightRenderer->SetBackground(colors->GetColor3d("LightSlateGray").GetData());

	// 两个窗口的摄像机一致旋转

	rightRenderer->SetActiveCamera(leftRenderer->GetActiveCamera());
	// Render and interact
	renderWindow->Render();

	renderWindowInteractor->Start();

	return EXIT_SUCCESS;
}

void GenerateData(vtkPolyData* input)
{
	
	vtkNew<vtkSphereSource> sphereSource;
	sphereSource->Update();

	// Remove some cells
	vtkNew<vtkIdTypeArray> ids;
	ids->SetNumberOfComponents(1);

	// Set values
	ids->InsertNextValue(2);
	ids->InsertNextValue(10);

	vtkNew<vtkSelectionNode> selectionNode;
	selectionNode->SetFieldType(vtkSelectionNode::CELL);
	selectionNode->SetContentType(vtkSelectionNode::INDICES);
	selectionNode->SetSelectionList(ids);
	selectionNode->GetProperties()->Set(vtkSelectionNode::INVERSE(),
		1); // invert the selection

	vtkNew<vtkSelection> selection;
	selection->AddNode(selectionNode);

	vtkNew<vtkExtractSelection> extractSelection;
	extractSelection->SetInputConnection(0, sphereSource->GetOutputPort());
	extractSelection->SetInputData(1, selection);
	extractSelection->Update();

	// In selection
	vtkNew<vtkDataSetSurfaceFilter> surfaceFilter;
	surfaceFilter->SetInputConnection(extractSelection->GetOutputPort());
	surfaceFilter->Update();

	input->ShallowCopy(surfaceFilter->GetOutput());
}
// Snippets
namespace {
	vtkSmartPointer<vtkPolyData> ReadPolyData(const char* fileName)
	{
		vtkSmartPointer<vtkPolyData> polyData;
		std::string extension =
			vtksys::SystemTools::GetFilenameExtension(std::string(fileName));
		if (extension == ".ply")
		{
			vtkNew<vtkPLYReader> reader;
			reader->SetFileName(fileName);
			reader->Update();
			polyData = reader->GetOutput();
		}
		else if (extension == ".vtp")
		{
			vtkNew<vtkXMLPolyDataReader> reader;
			reader->SetFileName(fileName);
			reader->Update();
			polyData = reader->GetOutput();
		}
		else if (extension == ".obj")
		{
			vtkNew<vtkOBJReader> reader;
			reader->SetFileName(fileName);
			reader->Update();
			polyData = reader->GetOutput();
		}
		else if (extension == ".stl")
		{
			vtkNew<vtkSTLReader> reader;
			reader->SetFileName(fileName);
			reader->Update();
			polyData = reader->GetOutput();
		}
		else if (extension == ".vtk")
		{
			vtkNew<vtkPolyDataReader> reader;
			reader->SetFileName(fileName);
			reader->Update();
			polyData = reader->GetOutput();
		}
		else if (extension == ".g")
		{
			vtkNew<vtkBYUReader> reader;
			reader->SetGeometryFileName(fileName);
			reader->Update();
			polyData = reader->GetOutput();
		}
		else
		{
			vtkNew<vtkSphereSource> source;
			source->Update();
			polyData = source->GetOutput();
		}
		return polyData;
	}
} // namespace
