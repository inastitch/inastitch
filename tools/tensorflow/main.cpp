// Created on 13.11.2020
// by Vincent Jordan

// TensorFlow Lite includes:
#include "edgetpu.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"

// See docs:
// https://www.tensorflow.org/lite/guide/inference#load_and_run_a_model_in_c
// https://www.tensorflow.org/lite/models/object_detection/overview

// OpenCv includes:
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

// Std includes:
#include <iostream>
#include <fstream>
#include <chrono>

int main(int /*argc*/, char** /*argv*/)
{
     cv::VideoCapture cap(0);

     if(!cap.isOpened())
     {
        std::cout << "Cannot open camera" << std::endl;
        return -1;
     }

     double camWidth = cap.get(cv::CAP_PROP_FRAME_WIDTH);
     double camHeight = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
     std::cout << "Opened camera at " << camWidth << "x" << camHeight << std::endl;

     std::cout << "Running with EdgeTPU " << edgetpu::EdgeTpuManager::GetSingleton()->Version() << std::endl;
     // Sets up the tpu_context.
     edgetpu::EdgeTpuManager::GetSingleton()->SetVerbosity(1);
     auto tpuContext = edgetpu::EdgeTpuManager::GetSingleton()->OpenDevice();
     if(!tpuContext->IsReady()) {
         std::cout << "EdgeTPU is not ready" << std::endl;
         return -1;
     }

     // Load model
     auto model = tflite::FlatBufferModel::BuildFromFile("mobilenet_v1_300x300.tflite");

     // Registers edge TPU custom op handler with Tflite resolver.
     tflite::ops::builtin::BuiltinOpResolver resolver;
     resolver.AddCustom(edgetpu::kCustomOp, edgetpu::RegisterCustomOp());

     // Make the interpreter
     std::unique_ptr<tflite::Interpreter> interpreter;
     if( tflite::InterpreterBuilder(*model, resolver)(&interpreter) != kTfLiteOk) {
         std::cerr << "Cannot create interpreter" << std::endl;
         return -1;
     }

     if (interpreter->AllocateTensors() != kTfLiteOk) {
        std::cerr << "Cannot allocate interpreter tensors" << std::endl;
        return -1;
     }

     // Load labels
     std::vector<std::string> labels;
     {
         std::ifstream in("coco_labels.txt");
         if(!in.is_open()) return -1;

         std::string str;
         while (std::getline(in, str))
         {
             if(str.size()>0) labels.push_back(str);
         }
         in.close();
     }

    while(true)
    {
        cv::Mat camFrame;
        const auto tStart = std::chrono::high_resolution_clock::now();
        {
            bool isFrameReady = cap.read(camFrame);

            if(!isFrameReady)
            {
                std::cout << "Failed to read frame" << std::endl;
                break;
            }

            assert(camWidth  == camFrame.cols);
            assert(camHeight == camFrame.rows);
        }
        const auto tRead = std::chrono::high_resolution_clock::now();

        static const auto tensorWidth  = 300;
        static const auto tensorHeight = 300;
        cv::Mat tensorFrame;
        {
            // copy image to input as input tensor
            cv::resize(camFrame, tensorFrame, cv::Size(tensorWidth, tensorHeight));
            memcpy(interpreter->typed_input_tensor<uchar>(0), tensorFrame.data, tensorFrame.total() * tensorFrame.elemSize());
        }
        const auto tResize = std::chrono::high_resolution_clock::now();

        {
            //interpreter->SetAllowFp16PrecisionForFp32(true);
            // Binds a context with a specific interpreter.
            interpreter->SetExternalContext(kTfLiteEdgeTpuContext, tpuContext.get());
            interpreter->SetNumThreads(1);

#if 0
            std::cout << "tensors size: " << interpreter->tensors_size() << "\n";
            std::cout << "nodes size: " << interpreter->nodes_size() << "\n";
            std::cout << "inputs: " << interpreter->inputs().size() << "\n";
            std::cout << "input(0) name: " << interpreter->GetInputName(0) << "\n";
            std::cout << "outputs: " << interpreter->outputs().size() << "\n";
#endif

            // run model
            if (interpreter->Invoke() != kTfLiteOk) {
                std::cerr << "Cannot invoke interpreter" << std::endl;
                return 1;
            }
        }
        const auto tInvoke = std::chrono::high_resolution_clock::now();

        {
            const float* detectionLocations = interpreter->tensor(interpreter->outputs()[0])->data.f;
            const float* detectionClasses   = interpreter->tensor(interpreter->outputs()[1])->data.f;
            const float* detectionScores    = interpreter->tensor(interpreter->outputs()[2])->data.f;
            const int    detectionsCount    = *interpreter->tensor(interpreter->outputs()[3])->data.f;

            const float confidenceThreshold = 0.5;
            for(int i = 0; i < detectionsCount; i++)
            {
                if(detectionScores[i] > confidenceThreshold)
                {
                    int det_index = (int)detectionClasses[i]+1;
                    float y1 = detectionLocations[4*i  ] * camHeight;
                    float x1 = detectionLocations[4*i+1] * camWidth;
                    float y2 = detectionLocations[4*i+2] * camHeight;
                    float x2 = detectionLocations[4*i+3] * camWidth;

                    cv::Rect rec((int)x1, (int)y1, (int)(x2 - x1), (int)(y2 - y1));
                    rectangle(camFrame, rec, cv::Scalar(0, 0, 255), 1, 8, 0);
                    putText(camFrame, cv::format("%s", labels[det_index].c_str()), cv::Point(x1, y1-5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1, 8, 0);
                }
            }
        }
        const auto tOverlay = std::chrono::high_resolution_clock::now();

        const auto readTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(tRead - tStart).count();
        const auto resizeTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(tResize - tRead).count();
        const auto invokeTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(tInvoke - tResize).count();
        const auto overlayTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(tOverlay - tInvoke).count();

        const auto frameTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(tOverlay-tStart).count();
        std::cout << "readTime=" << readTimeUs << "us"
                  << ", resizeTime=" << resizeTimeUs << "us"
                  << ", invokeTime=" << invokeTimeMs << "ms"
                  << ", overlayTime=" << overlayTimeUs << "us"
                  << ", frameTime=" << frameTimeMs << "ms" << std::endl;

        imshow("webcam", camFrame);

        // wait 1ms for a key
        if(cv::waitKey(1) == 27) {
            std::cout << "Exit" << std::endl;
            break;
        }
    }

    cap.release();

    return 0;
}
