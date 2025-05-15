#include"ImageProcess.hpp"


static const double RAD2DEG = 180.0 / M_PI;

float GrabData::get_phi(cv::Point2f p1, cv::Point2f p2) {
    float angle = 0;
    if (p1.y > p2.y) {
        float distY = p1.y - p2.y;
        float distX = p1.x - p2.x;
        angle = 180 - atan2(distY, distX) * 180 / 3.14159;
    } else {
        float distY = p2.y - p1.y;
        float distX = p2.x - p1.x;
        angle = 180 - atan2(distY, distX) * 180 / 3.14159;
    }
    return angle;
}

cv::Mat GrabData::findLargesrArea(cv::Mat srcImage) {
    cv::Mat temp;
    cv::Mat labels;
    srcImage.copyTo(temp);//1. 标记连通域
    int n_comps = connectedComponents(temp, labels, 4, CV_16U);
    std::vector<int> histogram_of_labels;
    for (int i = 0; i < n_comps; i++)//初始化labels的个数为0
    {
        histogram_of_labels.push_back(0);
    }
    int rows = labels.rows;
    int cols = labels.cols;
    for (int row = 0; row < rows; row++) //计算每个labels的个数--即连通域的面积
    {
        for (int col = 0; col < cols; col++) {
            histogram_of_labels.at(labels.at<unsigned short>(row, col)) += 1;
        }
    }
    histogram_of_labels.at(0) = 0; //将背景的labels个数设置为0//2. 计算最大的连通域labels索引
    int maximum = 0;
    int max_idx = 0;
    for (int i = 0; i < n_comps; i++) {
        if (histogram_of_labels.at(i) > maximum) {
            maximum = histogram_of_labels.at(i);
            max_idx = i;
        }
    }//3. 将最大连通域标记为255，并将其他连通域置0
    for (int row = 0; row < rows; row++) {
        for (int col = 0; col < cols; col++) {
            if (labels.at<unsigned short>(row, col) == max_idx) {
                labels.at<unsigned short>(row, col) = 255;
            } else { labels.at<unsigned short>(row, col) = 0; }
        }
    }//4. 将图像更改为CV_8U格式
    labels.convertTo(labels, CV_8U);
    return labels;
}

void GrabData::Pretreatment(cv::Mat &srcImage) {
    //三通道转单通道灰度图
    //cv::Mat ImageSingle;
    //cvtColor(srcImage, ImageSingle, cv::COLOR_RGB2GRAY);

    // 阈值分割
    cv::Mat ImageBlack;
    threshold(srcImage, ImageBlack, 128, 256, cv::THRESH_BINARY);

    // 提取最大区域
    cv::Mat areaMax = findLargesrArea(ImageBlack);

    // 膨胀
    cv::Mat element = getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    dilate(areaMax, areaMax, element);

    // 腐蚀
    cv::Mat element2 = getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    erode(areaMax, srcImage, element2);
}

//新
//获取抓取点center和phi
cv::Mat GrabData::getCP(cv::Mat &srcImage, cv::Mat &ImageDealt) {
    // 轮廓提取
    cv::Mat dstImage_3(ImageDealt.size(), CV_8UC3, cv::Scalar(0));
    std::vector<std::vector<cv::Point>> contours;          //保存轮廓
    findContours(ImageDealt, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,
                 cv::Point());   //RETR_EXTERNAL只检测外轮廓,CHAIN_APPROX_NONE把轮廓上所有点储存， Point()偏移量0
    std::vector<std::vector<cv::Point>> contours_poly(contours.size());  //保存逼近轮廓
    for (int i = 0; i < contours.size(); i++) {
        //epsilon==1.5
        approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 2.2, true);          //用指定精度逼近多边形曲线 ，越小精度越高
        drawContours(dstImage_3, contours_poly, i, cv::Scalar(230, 130, 255), 1, cv::LINE_AA);
    }

    // 构建容器Dist，存放轮廓线的模
    std::vector<double> Dist;
    for (int j = 0; j < contours_poly[0].size() - 1; j++) {
        double a, b, dist;
        a = contours_poly[0][j].x - contours_poly[0][j + 1].x;
        b = contours_poly[0][j].y - contours_poly[0][j + 1].y;
        a = pow(a, 2);
        b = pow(b, 2);
        dist = sqrt(a + b);
        Dist.push_back(dist);
        //cout << "轮廓线" << j + 1 << "的模: " << dist << endl;
    }
    // 寻找最大模的轮廓线
//    double maxValue = *max_element(Dist.begin(), Dist.end());
    int maxPosition = max_element(Dist.begin(), Dist.end()) - Dist.begin();
    //cout << "轮廓模最大：" << maxValue << "   "<<"索引: "<< maxPosition<<endl;
    cv::Point2f p1, p2;
    p1 = contours_poly[0][maxPosition];
    p2 = contours_poly[0][maxPosition + 1];
    //cout << p1 << endl;
    //cout << p2 << endl;
    m_result.rz = get_phi(p1, p2);

    //1.找出最长线段ab
    //2.求出其他点到该线段的垂直距离，剔除最大距离的几个点
    //3.求出最大距离点c，到线段ab两个端点的距离，设距离最小的端点为a
    //4.以a为起点，朝着垂直于ab且与其他点同侧的方向延长一个点d，放入容器中
    //5.求剩下点的包围矩形
    std::vector<double> perpendicularDistanceVec;
    for (int i = 0; i < contours_poly[0].size(); i++) {
        double ppDist = perpendicularDistance(p1, p2, contours_poly[0][i]);
        perpendicularDistanceVec.push_back(ppDist);
    }

    std::vector<int> deletemark;
    double maxPPD = *max_element(perpendicularDistanceVec.begin(), perpendicularDistanceVec.end());

    //maxPPD>35为h型，否则为v型,自动调整限制
    float limitDist = 0;
    if (maxPPD > 35) {
        limitDist = 8.5;
    } else {
        limitDist = 25;
    }
    for (int i = 0; i < perpendicularDistanceVec.size(); i++) {
        //cout << perpendicularDistanceVec[i] << endl;
        if (perpendicularDistanceVec[i] > limitDist) {
            deletemark.push_back(i);
        }
    }
    sort(deletemark.begin(), deletemark.end(), std::greater<int>());  //降序排序

    for (int i = 0; i < deletemark.size(); i++) {
        contours_poly[0].erase(contours_poly[0].begin() + deletemark[i]);
        perpendicularDistanceVec.erase(perpendicularDistanceVec.begin() + deletemark[i]);
    }

    cv::RotatedRect minRect = cv::minAreaRect(contours_poly[0]);
    ShrinkRotatedRect(minRect, 0.5);
    cv::Point2f rectPoints[4];
    minRect.points(rectPoints);
    for (int i = 0; i < 4; i++) {
        cv::line(srcImage, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(0, 255, 0), 1.5);
    }

    m_result.center = minRect.center;

    cv::circle(srcImage, m_result.center, 2, cv::Scalar(0, 0, 255), -1);

    std::string savePath = "../results";

    cv::imwrite(savePath + "/ImageCenter.png", srcImage);

    return srcImage;
}

// 将图像坐标转换为世界坐标
cv::Point2f GrabData::ImgtRob(float k1, float k2, float k3, float k4, float b1, float b2, cv::Point ImCenter) {
    cv::Point2f RobCenter;
    RobCenter.x = float(ImCenter.x) * k1 + float(ImCenter.y) * k2 + b1;
    RobCenter.y = float(ImCenter.x) * k3 + float(ImCenter.y) * k4 + b2;
    return RobCenter;
}
//[RobCenter.x; RobCenter.y; 1] = [k1, k2, b1; k3, k4, b2;0,0,1 ] * [ImCenter.x; ImCenter.y; 1]


cv::Point3f GrabData::convertItoW(cv::Point3f ImagePoint) {
    cv::Mat instri(3, 3, CV_32FC1, m_depth_intri);
    cv::Mat inv_instri;
    cv::invert(instri, inv_instri);
    cv::Mat pixel = (cv::Mat_<float>(3, 1) << ImagePoint.x, ImagePoint.y, 1);
    cv::Mat camera = ImagePoint.z * inv_instri * pixel;
    cv::Mat world = m_Rotation * camera + m_Translation;
    cv::Point3f WorldCoordinates;
    WorldCoordinates.x = world.at<float>(0, 0);
    WorldCoordinates.y = world.at<float>(1, 0);
    WorldCoordinates.z = world.at<float>(2, 0);
    m_result.x = WorldCoordinates.x;
    m_result.y = WorldCoordinates.y;
    m_result.z = WorldCoordinates.z;
    return WorldCoordinates;
}

float GrabData::getDepth(const cv::Mat& depthImage, cv::Rect depthRoi)  //1190-depth
{
    cv::Mat imdepth = depthImage.clone();   //对depthimage深拷贝
    cv::Mat roidepth = imdepth(depthRoi);

    std::vector<float> depthData;               //定义容器存放roi区域中原始深度值
    float meanDepth;                       //存放平均深度

    //遍历整个深度区域，将深度值存放到容器中
    for (int i = 0; i < roidepth.rows; i++) {
        for (int j = 0; j < roidepth.cols; j++) {
            if ((roidepth.at<uint16_t>(i, j) * 1.f) == 0)
                continue;
            else {
                depthData.push_back(roidepth.at<uint16_t>(i, j) * 1.f);
                //cout << roidepth.at<uint16_t>(i, j) * 1.f << endl;
            }
        }
    }// 1345 1346 1346 1345 1346 1346 1345 1356 1357 1356
    //cout << "----------------------" << endl;
    std::vector<float> depthDivided;         //存放被10整除后的深度
    for (float &it: depthData) {
        depthDivided.push_back(floor(it / 10));
    }

    sort(depthDivided.begin(), depthDivided.end());
    int length = depthDivided.size();

    int i = 0;
    int MaxCount = 1;
    int index = 0;

    while (i < length)        //遍历
    {
        int count = 1;
        int j;
        for (j = i; j < length - 1; j++) {
            if (depthDivided[j] == depthDivided[j + 1])//存在连续两个数相等，则众数+1
            {
                count++;
            } else {
                break;
            }
        }
        if (MaxCount < count) {
            MaxCount = count;    //当前最大众数
            index = j;          //当前众数标记位置
        }
        ++j;
        i = j;//位置后移到下一个未出现的数字
    }

    int mod = depthDivided[index];     // 出现频率最多的深度（/10）
    //cout << "mod = " << mod << "  MaxCount = " << MaxCount << endl;

    std::vector<float> depthFinal;     // 存放频率最多的一组深度
    for (float &it: depthData) {
        int temp = floor(it / 10);
        if (temp == mod) {
            depthFinal.push_back(it);
        } else
            continue;
    }

    //for (vector<float>::iterator it = depthFinal.begin(); it != depthFinal.end(); it++)
    //{
    //	cout << *it << endl;
    //}

    float sumDepth = accumulate(begin(depthFinal), end(depthFinal), 0.0);   // accumulate函数就是求vector和的函数；
    meanDepth = sumDepth / depthFinal.size();                       // 求均值
    return meanDepth;
}

void GrabData::PreAllmasks(std::vector<cv::Mat> &masksVec) {
    for (auto &mask: masksVec) {
        Pretreatment(mask);
    }
}

std::vector<int> GrabData::getArea(std::vector<cv::Mat> &masksVec) {
    std::vector<int> areaVec;
    for (int i = 0; i < masksVec.size(); i++) {
        int whitePixels = cv::countNonZero(masksVec[i] == 255);
//        std::cout << "mask" + std::to_string(i + 1) + " area= " << whitePixels << std::endl;
        areaVec.push_back(whitePixels);
    }
    return areaVec;
}

void GrabData::updateDepth(cv::Mat &depth) {
    cv::Rect rect(250, 200, 850, 700);
    for (int i = 0; i < depth.rows; i++) {
        for (int j = 0; j < depth.cols; j++) {
            if (!rect.contains(cv::Point(j, i))) {
                depth.ptr<unsigned short>(i)[j] = 0;
            }
        }
    }
}

Contours GrabData::getContours(cv::Mat mask) {
    Contours contours;          //保存轮廓
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,
                     cv::Point());   //RETR_EXTERNAL只检测外轮廓,CHAIN_APPROX_NONE把轮廓上所有点储存， Point()偏移量0

    return contours;
}

std::vector<Contours> GrabData::saveAllContours(std::vector<cv::Mat> &masksVec, int maskNum) {
    std::vector<Contours> contoursVec;
    for (int i = 0; i < maskNum; i++) {
        Contours contoursSingle = getContours(masksVec[i]);
        contoursVec.push_back(contoursSingle);
    }
    return contoursVec;
}

cv::Rect GrabData::ExtractTargetArea(Contours contours, const cv::Mat& outColor) {
    if (cv::contourArea(contours[0]) > 0) {
        // 根据轮廓创建目标区域的矩形边界框
        cv::Rect boundingBox = cv::boundingRect(contours[0]);
        // 放大边界框
        cv::Size size(30, 30);
        boundingBox = rectCenterScale(boundingBox, size);

        // 在原始图像上提取目标区域
        cv::Mat targetRegion = outColor(boundingBox);

        return boundingBox;
    }
    return {};
}

cv::Rect GrabData::rectCenterScale(cv::Rect rect, cv::Size size) {
    rect = rect + size;
    cv::Point pt;
    pt.x = cvRound(size.width / 2.0);
    pt.y = cvRound(size.height / 2.0);
    return (rect - pt);
}

cv::Mat GrabData::ColorFilter(cv::Mat outDepth, cv::Mat outColor) {
    cv::Rect rect(250, 200, 850, 700);
    std::vector<float> depthData;               //定义容器存放roi区域中原始深度值
    for (int i = 0; i < outDepth.rows; i++) {
        for (int j = 0; j < outDepth.cols; j++) {
            if (!rect.contains(cv::Point(j, i))) {
                continue;
            }
            if (outDepth.ptr<unsigned short>(i)[j] == 0) {
                continue;
            }
            depthData.push_back(outDepth.ptr<unsigned short>(i)[j]);
        }
    }
    std::vector<float> depthDivided;         //存放被10整除后的深度
    for (float &it: depthData) {
        depthDivided.push_back(floor(it / 10));
    }

    // 使用 std::min_element 找出容器中的最小值(以10为一组）
    auto minElement = min_element(depthDivided.begin(), depthDivided.end());

    // 检查最小元素是否找到
    int minDepth = 0;
    if (minElement != depthDivided.end()) {
        minDepth = *minElement;
    }

    std::vector<float> filteredDepth;   //存放筛选后的深度
    int numPixel = 0;

    // 根据深度点数先过滤掉偶然性最高点
    if (minDepth > 0) {
        while (numPixel < 100)         //最顶层深度值点过少直接舍弃，再向下采样一组
        {
            filteredDepth.clear();  //先清空容器

            for (float &it: depthData) {
                int temp = floor(it / 10);
                if (temp == minDepth) {
                    filteredDepth.push_back(it);
                } else
                    continue;
            }
            numPixel = filteredDepth.size();
            minDepth++;
        }
    }

    // 初步确定过滤深度
    float sumDepth = accumulate(begin(filteredDepth), end(filteredDepth), 0.0);
    float meanfilteredDepth = sumDepth / filteredDepth.size();
    std::cout << "meanfilteredDepth = " << meanfilteredDepth << std::endl;

    int area_ = 0;
    float depthBias = meanfilteredDepth + 15;

    cv::Mat color;
    int minArea_filtered = 2500;

    // 再根据过滤后图像是否存在完整目标决定是否向下采样
    while (area_ < minArea_filtered) {
        std::cout << "depthBias = " << depthBias << std::endl;
        color.create(outColor.size(), outColor.type());

        // 过滤彩图
        for (int i = 0; i < outDepth.rows; i++) {
            for (int j = 0; j < outDepth.cols; j++) {
                if (!rect.contains(cv::Point(j, i))) {
                    continue;
                }
                if (outDepth.ptr<unsigned short>(i)[j] == 0) {
                    continue;
                }
                if (outDepth.ptr<unsigned short>(i)[j] < depthBias) {
                    color.at<cv::Vec3b>(i, j) = outColor.at<cv::Vec3b>(i, j);
                }
            }
        }

        cv::Mat grayImage;
        cv::cvtColor(color, grayImage, cv::COLOR_BGR2GRAY);

        // 阈值分割
        cv::Mat thresholdImage;
        threshold(grayImage, thresholdImage, 1, 255, cv::THRESH_BINARY);

        // 查找连通域
        cv::Mat labels, stats, centroids;
        int numLabels = connectedComponentsWithStats(thresholdImage, labels, stats, centroids);

        // 计算最大连通域面积
        int maxArea = 0;
        for (int i = 1; i < numLabels; i++) {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            if (area > maxArea) {
                maxArea = area;
            }
        }
        area_ = maxArea;
        std::cout << "Max connected component area: " << maxArea << std::endl;
        depthBias = depthBias + 3;
    }

    return color;
}

double GrabData::perpendicularDistance(const cv::Point2f &A, const cv::Point2f &B, const cv::Point2f &C) {
    cv::Point2f AB = B - A;

    cv::Point2f AC = C - A;

    double dotProduct = AC.dot(AB);
    double lengthSquared = AB.dot(AB);
    double t = dotProduct / lengthSquared;

    cv::Point2f P = A + t * AB;

    double distance = cv::norm(C - P);

    return distance;
}

void GrabData::ShrinkRotatedRect(cv::RotatedRect &rect, double alpha) {
    // 确保 alpha 是一个有效的比例
    if (alpha <= 0 || alpha > 1) {
        throw std::invalid_argument("Alpha must be in the range (0, 1].");
    }

    // 缩小宽度和高度
    rect.size.width *= alpha;
    rect.size.height *= alpha;

    // rect 的中心点和角度保持不变

}


std::vector<Eigen::Matrix4d>
GrabData::computeBHos(const std::vector<Eigen::Matrix4d> &cHos) {
    std::vector<Eigen::Matrix4d> bHos;
    bHos.reserve(cHos.size());

    for (const auto &cHo: cHos) {
        // 计算 gHo = gHc * cHo
        Eigen::Matrix4d bHo = m_bHc * cHo;
        bHos.push_back(bHo);
    }

    for (size_t i = 0; i < bHos.size(); ++i) {
        std::cout << "bHo[" << i << "] = \n" << bHos[i] << "\n\n";
    }

    return bHos;
}

Eigen::Matrix4d GrabData::computeBHo(const Eigen::Matrix4d &cHo) {
    Eigen::Matrix4d bHo = m_bHc * cHo;
    return bHo;
}

Eigen::Matrix4d GrabData::computeBHg(const Eigen::Matrix4d &bHo) {
    Eigen::Matrix4d bHg = bHo * m_oHg;
    return bHg;
}

Eigen::Matrix4d GrabData::computeBHg2(const Eigen::Matrix4d &bHo) {
    // 1. 提取原始旋转矩阵 R 和位移向量 t
    Eigen::Matrix3d R = bHo.block<3, 3>(0, 0);
    Eigen::Vector3d t = bHo.block<3, 1>(0, 3);

    // 2. 获取当前的 z 轴（注意：R 的第三列即为该轴）
    // 因为 R 是正交矩阵，所以 R*(0,0,1) 已经是单位向量
    Eigen::Vector3d z_axis = R * Eigen::Vector3d(0, 0, 1);

    // 3. 构造绕当前 z 轴旋转 90°（逆时针 90°）的旋转矩阵
    Eigen::Matrix3d Rz_world = Eigen::AngleAxisd(M_PI / 2.0, z_axis).toRotationMatrix();

    // 4. 计算新的旋转矩阵
    // 这里等价于在局部坐标系下右乘一个绕 z 轴旋转 -90°的变换：new_R = R * R_local
    // 而利用公式 Rz_world = R * R_local * R⁻¹，可以得到 new_R = Rz_world * R
    Eigen::Matrix3d R_new = Rz_world * R;

    // 5. 计算新的位移向量：沿当前 z 轴负方向平移 125 个单位
    Eigen::Vector3d t_new = t - 125.0 * z_axis;

    // 6. 构造新的齐次变换矩阵 new_cHo
    Eigen::Matrix4d bHg = Eigen::Matrix4d::Identity();
    bHg.block<3, 3>(0, 0) = R_new;
    bHg.block<3, 1>(0, 3) = t_new;

    return bHg;
}


std::vector<double> GrabData::matrixToPose(const Eigen::Matrix4d &T) {
    // 平移分量
    double x = T(0, 3);
    double y = T(1, 3);
    double z = T(2, 3);

    // 旋转矩阵
    Eigen::Matrix3d R = T.block<3, 3>(0, 0);

    // 1) 计算 ry
    //    R(2,0) = -sin(ry)
    double sy = -R(2, 0); // -sin(ry)
    double cy = std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)); // cos(ry)
    double ry = std::atan2(sy, cy);

    // 2) 计算 rx
    //    R(2,1) = cos(ry)*sin(rx)
    //    R(2,2) = cos(ry)*cos(rx)
    double rx = std::atan2(R(2, 1), R(2, 2));

    // 3) 计算 rz
    //    R(0,0) = cos(rz)*cos(ry)
    //    R(1,0) = sin(rz)*cos(ry)
    double rz = std::atan2(R(1, 0), R(0, 0));

    // 转换为角度
    double rx_deg = rx * RAD2DEG;
    double ry_deg = ry * RAD2DEG;
    double rz_deg = rz * RAD2DEG;

    return {x, y, z, rx_deg, ry_deg, rz_deg};
}

// 将单个浮点轮廓转换为整数轮廓
std::vector<cv::Point> floatContourToInt(const std::vector<cv::Point2f> &floatContour) {
    std::vector<cv::Point> intContour;
    intContour.reserve(floatContour.size());
    for (const auto &pt: floatContour) {
        // 使用 cvRound(pt.x), cvRound(pt.y) 进行四舍五入
        intContour.emplace_back(cv::Point(cvRound(pt.x), cvRound(pt.y)));
    }
    return intContour;
}


void GrabData::drawSegmentsOnImage(const cv::Mat &image,
                                   const std::vector<std::vector<cv::Point2f>> &segments,
                                   std::vector<cv::Point2f> &centers,
                                   const cv::Scalar &outlineColor,
                                   const cv::Scalar &fillColor,
                                   int outlineThickness,
                                   double alpha) {

//    // 1. 创建一份图像副本，用于绘制填充效果
//    cv::Mat overlay;
//    image.copyTo(overlay);
//
//    // 2. 对每个轮廓进行填充
//    for (const auto &contourFloat: segments) {
//        if (contourFloat.empty()) continue;
//
//        // 转成整数坐标
//        std::vector<cv::Point> contourInt = floatContourToInt(contourFloat);
//
//        // fillPoly 接收的形参是 std::vector<std::vector<cv::Point>>
//        std::vector<std::vector<cv::Point>> pts{ contourInt };
//        cv::fillPoly(overlay, pts, fillColor);
//    }
//
//    // 3. 将填充结果与原图混合，实现半透明填充效果
//    cv::addWeighted(overlay, alpha, image, 1.0 - alpha, 0, image);
//
//    // 4. 绘制轮廓线
//    for (const auto &contourFloat: segments) {
//        if (contourFloat.empty()) continue;
//
//        std::vector<cv::Point> contourInt = floatContourToInt(contourFloat);
//        std::vector<std::vector<cv::Point>> pts{ contourInt };
//        cv::polylines(image, pts, true, outlineColor, outlineThickness);
//    }
//
//    // 5. 绘制质心
//    int radius = 3;
//    cv::Scalar centerColor(0, 0, 255);
    for (const auto &contourFloat: segments) {
        if (contourFloat.empty()) continue;

        // 计算轮廓的图像矩 (OpenCV 对 float 类型的 points 也兼容)
        cv::Moments mu = cv::moments(contourFloat, false);
        if (mu.m00 != 0) {
            int cx = static_cast<int>(mu.m10 / mu.m00);
            int cy = static_cast<int>(mu.m01 / mu.m00);
            cv::Point center(cx, cy);
            centers.emplace_back(cx, cy);  // 质心保存在 centers
//            cv::circle(image, center, radius, centerColor, -1);
        }
    }

//    cv::imwrite("../results/box1.png", image);
}


cv::Mat GrabData::imagePointToCamera(const cv::Mat &uvd) {
    float fx = m_cameraIntrinsic.at<float>(0, 0);  // f_x
    float fy = m_cameraIntrinsic.at<float>(1, 1);  // f_y
    float cx = m_cameraIntrinsic.at<float>(0, 2);  // c_x
    float cy = m_cameraIntrinsic.at<float>(1, 2);  // c_y

    float u = uvd.at<float>(0, 0);
    float v = uvd.at<float>(1, 0);
    float d = uvd.at<float>(2, 0);

    // 将 (u, v, d) 投影回相机坐标系 (x, y, z)
    float x = (u - cx) * d / fx;
    float y = (v - cy) * d / fy;
    float z = d;

    // 封装成 3×1 的结果返回
    cv::Mat xyz = (cv::Mat_<float>(3, 1) << x, y, z);
    return xyz;
}

cv::Mat GrabData::cameraPointToBase(const cv::Mat &xyz) {
    float x_c = xyz.at<float>(0, 0);
    float y_c = xyz.at<float>(1, 0);
    float z_c = xyz.at<float>(2, 0);

    Eigen::Vector4d p_c(static_cast<double>(x_c),
                        static_cast<double>(y_c),
                        static_cast<double>(z_c),
                        1.0);

    // 将点从相机坐标系转换到基座坐标系
    Eigen::Vector4d p_b = m_bHc * p_c;

    cv::Mat xyz_base = (cv::Mat_<float>(3, 1) << static_cast<float>(p_b(0)),
            static_cast<float>(p_b(1)),
            static_cast<float>(p_b(2)));
    return xyz_base;

}

cv::Mat GrabData::cameraPointToImage(float x, float y, float z) {

    // 从相机内参矩阵中读取参数
    float fx = m_cameraIntrinsic.at<float>(0, 0);
    float fy = m_cameraIntrinsic.at<float>(1, 1);
    float cx = m_cameraIntrinsic.at<float>(0, 2);
    float cy = m_cameraIntrinsic.at<float>(1, 2);

    // 注意：z 应大于0，否则投影无意义
    if (z <= 0.0f) {
        std::cerr << "Error: z must be positive for valid projection." << std::endl;
        return {};
    }

    // 根据针孔相机模型进行投影计算
    float u = fx * (x / z) + cx;
    float v = fy * (y / z) + cy;

    // 构造 2×1 的结果矩阵 (u, v)
    cv::Mat uv = (cv::Mat_<float>(2, 1) << u, v);
    return uv;

}

cv::Mat GrabData::cameraPointToImage(const cv::Mat &xyz) {
    float x = xyz.at<float>(0, 0);
    float y = xyz.at<float>(1, 0);
    float z = xyz.at<float>(2, 0);

    float fx = m_cameraIntrinsic.at<float>(0, 0);
    float fy = m_cameraIntrinsic.at<float>(1, 1);
    float cx = m_cameraIntrinsic.at<float>(0, 2);
    float cy = m_cameraIntrinsic.at<float>(1, 2);

    if (z <= 0.0f) {
        std::cerr << "Error: z must be positive for valid projection." << std::endl;
        return {};
    }

    float u = fx * (x / z) + cx;
    float v = fy * (y / z) + cy;

    cv::Mat uv = (cv::Mat_<float>(2, 1) << u, v);
    return uv;
}


void GrabData::setTranslation(Eigen::Matrix4d &cHo, const cv::Mat &xyz) {
    // 检查输入矩阵是否为 3×1
    if (xyz.rows != 3 || xyz.cols != 1) {
        std::cerr << "Error: Input xyz must be a 3x1 matrix." << std::endl;
        return;
    }

    // 将 cv::Mat 中的值赋值到 cHo 的平移部分 (最后一列的前三个元素)
    cHo(0, 3) = static_cast<double>(xyz.at<float>(0, 0));
    cHo(1, 3) = static_cast<double>(xyz.at<float>(1, 0));
    cHo(2, 3) = static_cast<double>(xyz.at<float>(2, 0));
}


void GrabData::drawObjectPose(cv::Mat &image, const Eigen::Matrix4d &cHo, double axisLength) {
    // 1. 在物体坐标系下定义原点及各轴端点（齐次坐标）
    Eigen::Vector4d origin(0, 0, 0, 1);
    Eigen::Vector4d xAxis(axisLength, 0, 0, 1);
    Eigen::Vector4d yAxis(0, axisLength, 0, 1);
    Eigen::Vector4d zAxis(0, 0, axisLength, 1);

    // 2. 利用 cHo 将这些点转换到相机坐标系下
    //    假设 cHo 已将物体坐标系下的点映射到相机坐标系
    Eigen::Vector4d origin_cam = cHo * origin;
    Eigen::Vector4d x_cam = cHo * xAxis;
    Eigen::Vector4d y_cam = cHo * yAxis;
    Eigen::Vector4d z_cam = cHo * zAxis;

    // 3. 提取转换后的 3D 坐标 (前三个分量)
    Eigen::Vector3d origin3d = origin_cam.head<3>();
    Eigen::Vector3d x3d = x_cam.head<3>();
    Eigen::Vector3d y3d = y_cam.head<3>();
    Eigen::Vector3d z3d = z_cam.head<3>();

    // 4. 利用相机内参将相机坐标系下的点投影到图像平面
    //    假设 m_cameraIntrinsic 为 3×3 的 CV_32F 矩阵，格式为 [fx 0 cx; 0 fy cy; 0 0 1]
    float fx = m_cameraIntrinsic.at<float>(0, 0);
    float fy = m_cameraIntrinsic.at<float>(1, 1);
    float cx = m_cameraIntrinsic.at<float>(0, 2);
    float cy = m_cameraIntrinsic.at<float>(1, 2);

    auto projectPoint = [fx, fy, cx, cy](const Eigen::Vector3d &p) -> cv::Point {
        // 如果 z<=0 表示点在相机后方，此时投影无意义
        if (p(2) <= 0)
            return cv::Point(-1, -1);
        float u = fx * (p(0) / p(2)) + cx;
        float v = fy * (p(1) / p(2)) + cy;
        return cv::Point(static_cast<int>(std::round(u)), static_cast<int>(std::round(v)));
    };

    cv::Point originPt = projectPoint(origin3d);
    cv::Point xPt = projectPoint(x3d);
    cv::Point yPt = projectPoint(y3d);
    cv::Point zPt = projectPoint(z3d);

    // 5. 若原点或任意轴端点投影不在相机前方，则不绘制
    if (originPt.x < 0 || xPt.x < 0 || yPt.x < 0 || zPt.x < 0) {
        std::cerr << "Warning: Some projected points are not visible (behind m_camera)." << std::endl;
        return;
    }

    // 6. 在图像上绘制三坐标轴：
    //    X轴：红色, Y轴：绿色, Z轴：蓝色
    cv::arrowedLine(image, originPt, xPt, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    cv::arrowedLine(image, originPt, yPt, cv::Scalar(0, 255, 0), 2, cv::LINE_AA);
    cv::arrowedLine(image, originPt, zPt, cv::Scalar(255, 0, 0), 2, cv::LINE_AA);
}


Eigen::Matrix4d GrabData::computeNewcHo(const Eigen::Matrix4d &cHo) {

    // 1. 提取原始旋转矩阵和平移向量
    Eigen::Matrix3d R_old = cHo.block<3, 3>(0, 0);
    Eigen::Vector3d t = cHo.block<3, 1>(0, 3);

    // 2. 定义光轴：图像平面法向（指向图像外）在相机坐标系中通常为 (0,0,1)
    Eigen::Vector3d opticalAxis(0, 0, 1);

    // 3. 构造候选的附加旋转矩阵 R_adj
    std::vector<Eigen::Matrix3d> candidates;

    // 加入身份矩阵（无旋转）作为候选
    candidates.emplace_back(Eigen::Matrix3d::Identity());

    // 定义旋转角度（单位：弧度）
    double rad90 = M_PI / 2.0;
    double rad180 = M_PI;
    double rad270 = 3.0 * M_PI / 2.0; // 等价于 -pi/2

    // 绕 x 轴旋转的候选
    Eigen::Matrix3d Rx90 = Eigen::AngleAxisd(rad90, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Rx180 = Eigen::AngleAxisd(rad180, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Rx270 = Eigen::AngleAxisd(rad270, Eigen::Vector3d::UnitX()).toRotationMatrix();

    candidates.push_back(Rx90);
    candidates.push_back(Rx180);
    candidates.push_back(Rx270);

    // 绕 y 轴旋转的候选
    Eigen::Matrix3d Ry90 = Eigen::AngleAxisd(rad90, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Ry180 = Eigen::AngleAxisd(rad180, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Ry270 = Eigen::AngleAxisd(rad270, Eigen::Vector3d::UnitY()).toRotationMatrix();

    candidates.push_back(Ry90);
    candidates.push_back(Ry180);
    candidates.push_back(Ry270);

    // 4. 遍历候选附加旋转，选择使新 z 轴（R_old * R_adj 的第三列）与 opticalAxis 点积最大的那个
    double bestDot = -std::numeric_limits<double>::infinity();
    Eigen::Matrix3d best_R_adj = Eigen::Matrix3d::Identity();

    for (const auto &R_adj: candidates) {
        Eigen::Matrix3d candidate_R = R_old * R_adj;
        // 物体坐标系下的 z 轴在相机坐标系中的表达，即 candidate_R 的第三列
        Eigen::Vector3d candidate_z = candidate_R.col(2);
        double dotVal = candidate_z.dot(opticalAxis);
        if (dotVal > bestDot) {
            bestDot = dotVal;
            best_R_adj = R_adj;
        }
    }

    // 5. 计算新的旋转矩阵
    Eigen::Matrix3d R_new = R_old * best_R_adj;


//    // ---------------------------------------------------------------
//    // 6. 进一步保证 x 轴与 "从左到右" (1,0,0) 方向呈锐角 (点积 > 0)
//    //    如果点积 < 0，则绕 z 轴转 180 度反转 x、y 轴
//    // ---------------------------------------------------------------
//
//    // 新的 x 轴
//    Eigen::Vector3d x_axis_cam = R_new.col(0);
//    // 定义“从左到右”方向，在相机坐标系里我们假设是 (1,0,0)
//    Eigen::Vector3d leftToRight(1, 0, 0);
//
//    double dotX = x_axis_cam.dot(leftToRight);
//    if (dotX < 0) {
//        // 绕 z 轴转 180 度
//        Eigen::AngleAxisd rot180AroundZ(M_PI, Eigen::Vector3d::UnitZ());
//        Eigen::Matrix3d Rz180 = rot180AroundZ.toRotationMatrix();
//        // 更新 R_new
//        R_new = R_new * Rz180;
//    }

    // 7. 构造新的变换矩阵（平移保持不变）
    Eigen::Matrix4d new_cHo = Eigen::Matrix4d::Identity();
    new_cHo.block<3, 3>(0, 0) = R_new;
    new_cHo.block<3, 1>(0, 3) = t;


    return new_cHo;
}

int GrabData::chooseEffectorAmp(std::string &className) {
    if (className == "handle") {
        return 35;
    } else if (className == "socket01") {
        return 90;
    } else {
        return 100;
    }
}

int GrabData::chooseEffectorZOffset(std::string &className) {
    if (className == "handle") {
        return 20;
    } else if (className == "socket01") {
        return 35;
    } else {
        return 55;
    }
}

cv::Point2f GrabData::getHandleCenter(std::vector<cv::Point2f> contour, const cv::Mat &color) {
    // 构建容器Dist，存放轮廓线的模
    std::vector<double> Dist;
    for (int j = 0; j < contour.size() - 1; j++) {
        double a, b, dist;
        a = contour[j].x - contour[j + 1].x;
        b = contour[j].y - contour[j + 1].y;
        a = pow(a, 2);
        b = pow(b, 2);
        dist = sqrt(a + b);
        Dist.push_back(dist);
        //cout << "轮廓线" << j + 1 << "的模: " << dist << endl;
    }
    // 寻找最大模的轮廓线
//    double maxValue = *max_element(Dist.begin(), Dist.end());
    int maxPosition = max_element(Dist.begin(), Dist.end()) - Dist.begin();
    //cout << "轮廓模最大：" << maxValue << "   "<<"索引: "<< maxPosition<<endl;
    cv::Point2f p1, p2;
    p1 = contour[maxPosition];
    p2 = contour[maxPosition + 1];
    //cout << p1 << endl;
    //cout << p2 << endl;
    m_result.rz = get_phi(p1, p2);

    //1.找出最长线段ab
    //2.求出其他点到该线段的垂直距离，剔除最大距离的几个点
    //3.求出最大距离点c，到线段ab两个端点的距离，设距离最小的端点为a
    //4.以a为起点，朝着垂直于ab且与其他点同侧的方向延长一个点d，放入容器中
    //5.求剩下点的包围矩形
    std::vector<double> perpendicularDistanceVec;
    for (int i = 0; i < contour.size(); i++) {
        double ppDist = perpendicularDistance(p1, p2, contour[i]);
        perpendicularDistanceVec.push_back(ppDist);
    }

    std::vector<int> deletemark;
    double maxPPD = *max_element(perpendicularDistanceVec.begin(), perpendicularDistanceVec.end());

    //maxPPD>35为h型，否则为v型,自动调整限制
    float limitDist = 0;
    if (maxPPD > 35) {
        limitDist = 8.5;
    } else {
        limitDist = 25;
    }
    for (int i = 0; i < perpendicularDistanceVec.size(); i++) {
        //cout << perpendicularDistanceVec[i] << endl;
        if (perpendicularDistanceVec[i] > limitDist) {
            deletemark.push_back(i);
        }
    }
    sort(deletemark.begin(), deletemark.end(), std::greater<int>());  //降序排序

    for (int i = 0; i < deletemark.size(); i++) {
        contour.erase(contour.begin() + deletemark[i]);
        perpendicularDistanceVec.erase(perpendicularDistanceVec.begin() + deletemark[i]);
    }

    cv::RotatedRect minRect = cv::minAreaRect(contour);
    ShrinkRotatedRect(minRect, 0.5);
    cv::Point2f rectPoints[4];
    minRect.points(rectPoints);
    for (int i = 0; i < 4; i++) {
        cv::line(color, rectPoints[i], rectPoints[(i + 1) % 4], cv::Scalar(0, 255, 0), 1.5);
    }


    cv::circle(color, m_result.center, 2, cv::Scalar(0, 0, 255), -1);

    cv::imwrite("../results/ImageCenter.png", color);

    return minRect.center;

}


Eigen::Matrix4d GrabData::adjustObjectPose(
        const cv::Mat &image,
        const std::vector<cv::Point2f> &contourPoints,
        const Eigen::Matrix4d &T_obj_to_cam) {
    // A. 获取最小包围矩形
    cv::RotatedRect minRect = cv::minAreaRect(contourPoints);

    // 取出矩形的四个顶点
    cv::Point2f pts[4];
    minRect.points(pts);

    for (int i = 0; i < 4; i++) {
        cv::line(image, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
//    cv::imwrite("../results/rect.png", image);

    // 旋转90度
    // B. 计算矩形在物体坐标系 x / y 轴上的长度
    // 1) 提取两个边向量
    //    由于是闭合矩形，可用 pts[0]->pts[1], pts[1]->pts[2] 作为两个正交边
    cv::Point2f edge1 = pts[1] - pts[0]; // 对应 width 或 height
    cv::Point2f edge2 = pts[2] - pts[1]; // 对应另一条边

    // 转成 Eigen::Vector3d（假设在相机坐标系Z=0平面）
    Eigen::Vector3d e1_cam(edge1.x, edge1.y, 0.0);
    Eigen::Vector3d e2_cam(edge2.x, edge2.y, 0.0);

    // 2) 把“相机坐标系”下的向量转换到“物体坐标系”下
    //    T_obj_to_cam 是：物体点 -> 相机坐标系
    //    若要从相机坐标系向量 -> 物体坐标系向量，需要用其旋转部分的逆
    Eigen::Matrix3d R_obj_to_cam = T_obj_to_cam.block<3, 3>(0, 0);
    Eigen::Matrix3d R_cam_to_obj = R_obj_to_cam.transpose(); // 旋转的逆

    Eigen::Vector3d e1_obj = R_cam_to_obj * e1_cam;
    Eigen::Vector3d e2_obj = R_cam_to_obj * e2_cam;

    // 3) 物体坐标系下的 x / y 轴是 (1,0,0), (0,1,0)
    //    分别计算投影长度
    Eigen::Vector3d x_axis(1, 0, 0);
    Eigen::Vector3d y_axis(0, 1, 0);

    double length1_x = std::fabs(e1_obj.dot(x_axis));
    double length1_y = std::fabs(e1_obj.dot(y_axis));

    double length2_x = std::fabs(e2_obj.dot(x_axis));
    double length2_y = std::fabs(e2_obj.dot(y_axis));

    double width_obj = std::max(length1_x, length2_x);  // x 方向可能对应那个更长的投影
    double height_obj = std::max(length1_y, length2_y);  // y 方向可能对应那个更长的投影


    // C. 如果 x 长度 < y 长度，则绕 z 轴旋转 90°
    Eigen::Matrix4d T_obj_to_cam_new = T_obj_to_cam;  // 先复制
    if (width_obj < height_obj) {
        // 绕物体自身 z 轴转 90° => 对 T_obj_to_cam 的旋转部分右乘 Rz(90)
        Eigen::Matrix3d R_old = T_obj_to_cam.block<3, 3>(0, 0);

        Eigen::AngleAxisd rotateZ90(M_PI / 2.0, Eigen::Vector3d::UnitZ());
        Eigen::Matrix3d Rz90 = rotateZ90.toRotationMatrix();

        // 物体→相机 = R_old * ... => 右乘
        Eigen::Matrix3d R_new = R_old * Rz90;

        T_obj_to_cam_new.block<3, 3>(0, 0) = R_new;
        // 平移保持不变
    }

    return T_obj_to_cam_new;
}


Eigen::Matrix4d GrabData::adjustObjectPoseWithRect(
        const cv::Mat &image,
        const std::vector<cv::Point2f> &contourPoints,
        const Eigen::Matrix4d &T_obj_to_cam) {
    // A. 获取最小包围矩形，并绘制在图像上
    cv::RotatedRect minRect = cv::minAreaRect(contourPoints);
    cv::Point2f pts[4];
    minRect.points(pts);
    for (int i = 0; i < 4; i++) {
        cv::line(image, pts[i], pts[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
    }
//    cv::imwrite("../results/rect.png", image);

    // B. 计算矩形两条边向量
    cv::Point2f edge1 = pts[1] - pts[0];
    cv::Point2f edge2 = pts[2] - pts[1];

    // 根据边长选出较短的边作为短边
    float len1 = cv::norm(edge1);
    float len2 = cv::norm(edge2);
    cv::Point2f shortEdge = (len1 < len2) ? edge1 : edge2;

    // 计算短边相对于相机 x 轴的方向角（单位：弧度）
    double desired_angle = std::atan2(shortEdge.y, shortEdge.x);

    // C. 获取当前物体 y 轴在相机坐标系中的方向
    Eigen::Matrix3d R_old = T_obj_to_cam.block<3, 3>(0, 0);
    Eigen::Vector3d current_y = R_old.col(1);  // 物体的 y 轴
    double current_y_angle = std::atan2(current_y.y(), current_y.x());

    // 计算需要旋转的角度，使得物体 y 轴与短边平行
    double delta = desired_angle - current_y_angle;
    // 归一化 delta 到 [-pi, pi]
    while (delta > M_PI) delta -= 2 * M_PI;
    while (delta < -M_PI) delta += 2 * M_PI;

    // D. 构造绕 z 轴旋转 delta 的旋转矩阵
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    // 将原来的旋转部分右乘 Rz 得到新的旋转部分（平移保持不变）
    Eigen::Matrix4d T_obj_to_cam_new = T_obj_to_cam;
    T_obj_to_cam_new.block<3, 3>(0, 0) = R_old * Rz;

    return T_obj_to_cam_new;
}

