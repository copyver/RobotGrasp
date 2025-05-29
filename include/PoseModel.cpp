//
// Created by yhlever on 24-12-26.
//
#include "PoseModel.hpp"
#include "numeric"

bool init_numpy() {
    if (_import_array() < 0) {
        PyErr_Print();
        std::cerr << "Failed to initialize NumPy C API." << std::endl;
        return false;
    }
    return true;
}


ModelPose::ModelPose() {
    // set anaconda env
    std::string anacondaEnvPath = "/home/yhlever/anaconda3/envs/maskposenet";

    // set PYTHONHOME and PYTHONPATH
    const std::string &pythonHome = anacondaEnvPath;
    std::string pythonPath = anacondaEnvPath + "/lib/python3.10/site-packages";

    // set PYTHONHOME
    if (setenv("PYTHONHOME", pythonHome.c_str(), 1) != 0) {
        std::cerr << "Failed to set PYTHONHOME environment variable." << std::endl;
    }

    // set PYTHONPATH
    if (setenv("PYTHONPATH", pythonPath.c_str(), 1) != 0) {
        std::cerr << "Failed to set PYTHONPATH environment variable." << std::endl;
    }

    // set Python Home
    Py_SetPythonHome(Py_DecodeLocale(pythonHome.c_str(), nullptr));

    Py_Initialize();

    char exePath[PATH_MAX];
    ssize_t count = readlink("/proc/self/exe", exePath, PATH_MAX);
    if (count == -1) {
        std::cerr << "Failed to get executable path." << std::endl;
        return;
    }
    exePath[count] = '\0';

    char *lastSlash = strrchr(exePath, '/');
    if (lastSlash) *lastSlash = 0; // 删除最后一个斜杠及其之后的内容，以得到目录

    lastSlash = strrchr(exePath, '/');
    if (lastSlash) *lastSlash = 0; // 再次执行以得到上级目录

    //  'python'
    std::string pythonDir = std::string(exePath) + "/pytorch";

    std::string pythonCommand = "import sys; sys.path.append('";
    pythonCommand += pythonDir;
    pythonCommand += "')";

    PyRun_SimpleString(pythonCommand.c_str());
    init_numpy(); // 初始化 numpy array API

    PyEval_SaveThread();

}


ModelPose::~ModelPose() {
    if (Py_IsInitialized()) {
        Py_Finalize();
    }
}


PyObject *ModelPose::loadModel() {
    PyGILState_STATE gstate = PyGILState_Ensure();
    PyObject *pName = PyUnicode_FromString("pose_estimation");
    PyObject *pModule = PyImport_Import(pName); // 加载.py文件
    Py_DECREF(pName);
    if (pModule == nullptr) {
        PyErr_Print();
        std::cerr << "PyImport_ImportModule Fail!" << std::endl;
    }
    PyObject *pFunc = PyObject_GetAttrString(pModule, "predict_pose"); // 调用的函数名
    PyGILState_Release(gstate);
    return pFunc;
}


bool ModelPose::runPoseEstimation(PyObject *pFunc, cv::Mat colorImage, cv::Mat depthImage) {
    PyGILState_STATE gstate = PyGILState_Ensure();
    PyObject *pColor = matToNumpy(colorImage);
    PyObject *pDepth = matToNumpy(depthImage);

    if (!pColor || !pDepth) {
        return false;
    }

    PyObject *pArgs = PyTuple_New(2);
    PyTuple_SetItem(pArgs, 0, pColor);
    PyTuple_SetItem(pArgs, 1, pDepth);
    PyObject *pResult = PyObject_CallObject(pFunc, pArgs); // 调用 Python 函数
    Py_XDECREF(pArgs);

    if (!parseModelPoseResults(pResult)) {
        Py_XDECREF(pResult);
        return false;
    }

    Py_XDECREF(pResult);
    PyGILState_Release(gstate);
    return true;
}

PyObject *ModelPose::matToNumpy(const cv::Mat &mat) {
    if (!mat.data) {
        PyErr_SetString(PyExc_ValueError, "Null pointer in Mat");
        return nullptr;
    }

    int typenum;
    switch (mat.type()) {
        case CV_8UC1:
            typenum = NPY_UBYTE;
            break;
        case CV_8UC3:
            typenum = NPY_UBYTE;
            break;
        case CV_16UC1:
            typenum = NPY_UINT16;
            break;
        case CV_16UC3:
            typenum = NPY_UINT16;
            break;
        case CV_32FC1:
            typenum = NPY_FLOAT;
            break;
        case CV_32FC3:
            typenum = NPY_FLOAT;
            break;
        default:
            PyErr_SetString(PyExc_ValueError, "Unsupported data type in Mat");
            return nullptr;
    }

    npy_intp dims[3] = {mat.rows, mat.cols, mat.channels()};
    if (mat.channels() == 1) {
        PyObject *pArray = PyArray_SimpleNewFromData(2, dims, typenum, mat.data);
        return pArray;
    } else {
        PyObject *pArray = PyArray_SimpleNewFromData(3, dims, typenum, mat.data);
        return pArray;
    }
}


bool ModelPose::parseModelPoseResults(PyObject *pResult) {
    if (!pResult || !PyDict_Check(pResult)) {
        return false;
    }

    PyObject *pKey, *pValue;
    Py_ssize_t pos = 0;

    while (PyDict_Next(pResult, &pos, &pKey, &pValue)) {
        const char *keyStr = PyUnicode_AsUTF8(pKey);
        if (!keyStr) continue;

        // 如果 pValue 是 numpy 数组
        if (PyArray_Check(pValue)) {
            PyArrayObject *npArray = (PyArrayObject *) pValue;
            int ndim = PyArray_NDIM(npArray);
            npy_intp *shape = PyArray_SHAPE(npArray);
            float *data = (float *) PyArray_DATA(npArray);

            if (strcmp(keyStr, "pred_Rs") == 0 && ndim == 2) {
                // 填充 predRs
                result.predRs.resize(shape[0], std::vector<float>(shape[1]));
                for (npy_intp i = 0; i < shape[0]; ++i) {
                    for (npy_intp j = 0; j < shape[1]; ++j) {
                        result.predRs[i][j] = data[i * shape[1] + j];
                    }
                }
            } else if (strcmp(keyStr, "pred_Ts") == 0 && ndim == 2) {
                // 填充 predTs（单位放大 1000 倍）
                result.predTs.resize(shape[0], std::vector<float>(shape[1]));
                for (npy_intp i = 0; i < shape[0]; ++i) {
                    for (npy_intp j = 0; j < shape[1]; ++j) {
                        result.predTs[i][j] = data[i * shape[1] + j] * 1000;
                    }
                }
            } else if (strcmp(keyStr, "pose_scores") == 0 && ndim == 1) {
                // 填充 poseScores（对应 pred_scores 或 pose_scores）
                result.poseScores.resize(shape[0]);
                for (npy_intp i = 0; i < shape[0]; ++i) {
                    result.poseScores[i] = data[i];
                }
            }
        }
        else {
            if (strcmp(keyStr, "cls_names") == 0) {
                // cls_names 列表中存放字符串
                if (PyList_Check(pValue)) {
                    Py_ssize_t n = PyList_Size(pValue);
                    result.classNames.resize(n);
                    for (Py_ssize_t i = 0; i < n; i++) {
                        PyObject *pItem = PyList_GetItem(pValue, i);  // borrowed reference
                        if (PyUnicode_Check(pItem)) {
                            result.classNames[i] = PyUnicode_AsUTF8(pItem);
                        }
                    }
                }
            } else if (strcmp(keyStr, "seg_scores") == 0) {
                // seg_scores 为列表（浮点数）
                if (PyList_Check(pValue)) {
                    Py_ssize_t n = PyList_Size(pValue);
                    result.segScores.resize(n);
                    for (Py_ssize_t i = 0; i < n; i++) {
                        PyObject *pItem = PyList_GetItem(pValue, i);
                        result.segScores[i] = (float) PyFloat_AsDouble(pItem);
                    }
                }
            }else if (strcmp(keyStr, "seg_inference") == 0) {
                // seg_inference 为单个浮点数，转换为 float 类型
                if (PyFloat_Check(pValue)) {
                    result.segTime = (float) PyFloat_AsDouble(pValue);
                } else if (PyLong_Check(pValue)) {
                    result.segTime = (float) PyLong_AsLong(pValue);
                }
            } else if (strcmp(keyStr, "pose_inference") == 0) {
                // pose_inference 为单个浮点数，转换为 float 类型
                if (PyFloat_Check(pValue)) {
                    result.poseTime = (float) PyFloat_AsDouble(pValue);
                } else if (PyLong_Check(pValue)) {
                    result.poseTime = (float) PyLong_AsLong(pValue);
                }
            } else if (strcmp(keyStr, "adds") == 0) {
                // adds 为列表（浮点数）
                if (PyList_Check(pValue)) {
                    Py_ssize_t n = PyList_Size(pValue);
                    result.addMetrics.resize(n);
                    for (Py_ssize_t i = 0; i < n; i++) {
                        PyObject *pItem = PyList_GetItem(pValue, i);
                        result.addMetrics[i] = (float) PyFloat_AsDouble(pItem);
                    }
                }
            }// 添加对 segments 的解析，segments 为列表（表示分割点）
            else if (strcmp(keyStr, "segments") == 0) {
                if (PyList_Check(pValue)) {
                    Py_ssize_t nContours = PyList_Size(pValue);
                    result.segments.resize(nContours);
                    for (Py_ssize_t i = 0; i < nContours; i++) {
                        PyObject *pContour = PyList_GetItem(pValue, i);
                        if (PyArray_Check(pContour)) {
                            PyArrayObject *npContour = (PyArrayObject*) pContour;
                            int ndim = PyArray_NDIM(npContour);
                            if (ndim != 2) {
                                std::cerr << "[Warning] segments[" << i << "] ndarray ndim != 2" << std::endl;
                                continue;
                            }
                            npy_intp *shape = PyArray_SHAPE(npContour);
                            if (shape[1] != 2) {
                                std::cerr << "[Warning] segments[" << i << "] ndarray shape[1] != 2" << std::endl;
                                continue;
                            }
                            int numPoints = static_cast<int>(shape[0]);
                            auto *data = static_cast<float *>(PyArray_DATA(npContour));
                            std::vector<cv::Point2f> contour;
                            contour.reserve(numPoints);
                            for (int j = 0; j < numPoints; j++) {
                                float x = (data[j * 2]);
                                float y = (data[j * 2 + 1]);
                                contour.emplace_back(x, y);
                            }
                            result.segments[i] = contour;
                        } else {
                            std::cerr << "[Warning] segments[" << i << "] is not a numpy ndarray" << std::endl;
                        }
                    }
                }
            }
        }
    }

    result.cHos.reserve(result.predRs.size()); // 预分配空间

    for (size_t i = 0; i < result.predRs.size(); i++)
    {
        Eigen::Matrix4d cHo = Eigen::Matrix4d::Identity();

        cHo(0,0) = result.predRs[i][0];  // r00
        cHo(0,1) = result.predRs[i][1];  // r01
        cHo(0,2) = result.predRs[i][2];  // r02
        cHo(1,0) = result.predRs[i][3];  // r10
        cHo(1,1) = result.predRs[i][4];  // r11
        cHo(1,2) = result.predRs[i][5];  // r12
        cHo(2,0) = result.predRs[i][6];  // r20
        cHo(2,1) = result.predRs[i][7];  // r21
        cHo(2,2) = result.predRs[i][8];  // r22

        cHo(0,3) = result.predTs[i][0];  // tx
        cHo(1,3) = result.predTs[i][1];  // ty
        cHo(2,3) = result.predTs[i][2];  // tz

        result.cHos.push_back(cHo);
    }
    return true;
}


void ModelPose::printPoseResult() {
    std::cout << "================= Model Pose Result =================" << std::endl;

    const int colWidth = 12;
    std::cout << std::left
              << std::setw(colWidth) << "cls_name"
              << std::setw(colWidth) << "seg_score"
              << std::setw(colWidth) << "pose_score"
              << std::setw(colWidth) << "add"
              << std::endl;

    size_t n = result.classNames.size();

    float avgSeg = 0.f, avgPose = 0.f, avgAdd = 0.f;
    if (n > 0) {
        avgSeg = std::accumulate(result.segScores.begin(), result.segScores.end(), 0.f) / n;
        avgPose = std::accumulate(result.poseScores.begin(), result.poseScores.end(), 0.f) / n;
        avgAdd = std::accumulate(result.addMetrics.begin(), result.addMetrics.end(), 0.f) / n;
    }

    std::cout << std::left
              << std::setw(colWidth) << "all"
              << std::setw(colWidth) << std::fixed << std::setprecision(2) << avgSeg
              << std::setw(colWidth) << std::fixed << std::setprecision(2) << avgPose
              << std::setw(colWidth) << std::fixed << std::setprecision(2) << avgAdd
              << std::endl;

    for (size_t i = 0; i < n; ++i) {
        std::cout << std::left
                  << std::setw(colWidth) << result.classNames[i]
                  << std::setw(colWidth) << std::fixed << std::setprecision(2) << result.segScores[i]
                  << std::setw(colWidth) << std::fixed << std::setprecision(2) << result.poseScores[i]
                  << std::setw(colWidth) << std::fixed << std::setprecision(2) << result.addMetrics[i]
                  << std::endl;
    }


//    // 打印 predRs
//    std::cout << "predRs:" << std::endl;
//    for (size_t i = 0; i < result.predRs.size(); ++i) {
//        std::cout << "[ ";
//        for (size_t j = 0; j < result.predRs[i].size(); ++j) {
//            std::cout << std::fixed << std::setprecision(6) << result.predRs[i][j];
//            if (j < result.predRs[i].size() - 1) std::cout << ", ";
//        }
//        std::cout << " ]" << std::endl;
//    }
//
//    // 打印 predTs
//    std::cout << "predTs:" << std::endl;
//    for (size_t i = 0; i < result.predTs.size(); ++i) {
//        std::cout << "[ ";
//        for (size_t j = 0; j < result.predTs[i].size(); ++j) {
//            std::cout << std::fixed << std::setprecision(6) << result.predTs[i][j];
//            if (j < result.predTs[i].size() - 1) std::cout << ", ";
//        }
//        std::cout << " ]" << std::endl;
//    }

}
