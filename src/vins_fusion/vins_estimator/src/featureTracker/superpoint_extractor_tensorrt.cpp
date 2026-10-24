#include "superpoint_tensorrt.hpp"

#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

namespace
{

class TrtLogger : public nvinfer1::ILogger
{
public:
    void log(Severity severity, const char* msg) noexcept override
    {
        if (severity <= Severity::kWARNING)
            ROS_WARN_STREAM("[TensorRT] " << msg);
    }
};

TrtLogger g_trt_logger;

void checkCuda(cudaError_t status, const char* name)
{
    if (status != cudaSuccess)
        throw std::runtime_error(std::string(name) + " failed: " + cudaGetErrorString(status));
}

std::vector<char> readFile(const std::string& path)
{
    std::ifstream file(path.c_str(), std::ios::binary);
    if (!file)
        throw std::runtime_error("failed to open TensorRT engine: " + path);
    file.seekg(0, std::ios::end);
    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(static_cast<size_t>(size));
    if (!file.read(data.data(), size))
        throw std::runtime_error("failed to read TensorRT engine: " + path);
    return data;
}

bool hasDynamicDim(const nvinfer1::Dims& dims)
{
    for (int i = 0; i < dims.nbDims; ++i)
    {
        if (dims.d[i] < 0)
            return true;
    }
    return false;
}

bool sameDims(const nvinfer1::Dims& a, const nvinfer1::Dims& b)
{
    if (a.nbDims != b.nbDims)
        return false;
    for (int i = 0; i < a.nbDims; ++i)
    {
        if (a.d[i] != b.d[i])
            return false;
    }
    return true;
}

int64_t volume(const nvinfer1::Dims& dims)
{
    int64_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i)
    {
        if (dims.d[i] < 0)
            throw std::runtime_error("unresolved dynamic TensorRT shape");
        v *= dims.d[i];
    }
    return v;
}

size_t dtypeSize(nvinfer1::DataType dtype)
{
    switch (dtype)
    {
    case nvinfer1::DataType::kFLOAT:
        return 4;
    case nvinfer1::DataType::kHALF:
        return 2;
    case nvinfer1::DataType::kINT8:
        return 1;
    case nvinfer1::DataType::kINT32:
        return 4;
    case nvinfer1::DataType::kINT64:
        return 8;
    case nvinfer1::DataType::kBOOL:
        return 1;
    default:
        throw std::runtime_error("unsupported TensorRT data type");
    }
}

double readAsDouble(const void* data, nvinfer1::DataType dtype, int64_t index)
{
    switch (dtype)
    {
    case nvinfer1::DataType::kFLOAT:
        return static_cast<const float*>(data)[index];
    case nvinfer1::DataType::kINT32:
        return static_cast<const int32_t*>(data)[index];
    case nvinfer1::DataType::kINT64:
        return static_cast<double>(static_cast<const int64_t*>(data)[index]);
    default:
        throw std::runtime_error("unsupported output data type for SuperPoint parsing");
    }
}

class DynamicOutputAllocator : public nvinfer1::IOutputAllocator
{
public:
    ~DynamicOutputAllocator() override
    {
        if (device_)
            cudaFree(device_);
    }

    void* reallocateOutputAsync(const char* tensor_name,
                                void* current_memory,
                                uint64_t size,
                                uint64_t,
                                cudaStream_t) noexcept override
    {
        if (size <= capacity_ && device_)
            return device_;

        if (current_memory && current_memory != device_)
            cudaFree(current_memory);
        if (device_)
            cudaFree(device_);

        device_ = nullptr;
        cudaError_t status = cudaMalloc(&device_, static_cast<size_t>(size));
        if (status != cudaSuccess)
        {
            ROS_ERROR("cudaMalloc for dynamic output %s failed: %s", tensor_name, cudaGetErrorString(status));
            capacity_ = 0;
            return nullptr;
        }
        capacity_ = size;
        return device_;
    }

    void notifyShape(const char*, nvinfer1::Dims const& dims) noexcept override
    {
        dims_ = dims;
        has_shape_ = true;
    }

    void* device() const { return device_; }
    bool hasShape() const { return has_shape_; }
    nvinfer1::Dims dims() const { return dims_; }

private:
    void* device_ = nullptr;
    uint64_t capacity_ = 0;
    nvinfer1::Dims dims_;
    bool has_shape_ = false;
};

struct ScoredPoint
{
    cv::Point2f pt;
    float score;
};

class SuperPointBackend
{
public:
    virtual ~SuperPointBackend() {}
    virtual bool init(const SuperPointConfig& config) = 0;
    virtual bool infer(const cv::Mat& image, std::vector<ScoredPoint>& points) = 0;
};

struct TensorBuffer
{
    std::string name;
    bool is_input = false;
    bool dynamic_output = false;
    nvinfer1::Dims dims;
    nvinfer1::DataType dtype;
    void* device = nullptr;
    std::vector<unsigned char> host;
    DynamicOutputAllocator* allocator = nullptr;
};

class TrtSuperPointBackend : public SuperPointBackend
{
public:
    ~TrtSuperPointBackend() override {}

    bool init(const SuperPointConfig& config) override
    {
        try
        {
            std::vector<char> engine_data = readFile(config.engine_path);
            runtime_.reset(nvinfer1::createInferRuntime(g_trt_logger));
            if (!runtime_)
                throw std::runtime_error("failed to create TensorRT runtime");

            engine_.reset(runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size()));
            if (!engine_)
                throw std::runtime_error("failed to deserialize TensorRT engine");

            context_.reset(engine_->createExecutionContext());
            if (!context_)
                throw std::runtime_error("failed to create TensorRT context");

            input_name_.clear();
            keypoints_name_.clear();
            scores_name_.clear();
            const int nb_io = engine_->getNbIOTensors();
            for (int i = 0; i < nb_io; ++i)
            {
                std::string name = engine_->getIOTensorName(i);
                if (engine_->getTensorIOMode(name.c_str()) == nvinfer1::TensorIOMode::kINPUT)
                    input_name_ = name;
                else if (name.find("keypoints") != std::string::npos)
                    keypoints_name_ = name;
                else if (name.find("scores") != std::string::npos)
                    scores_name_ = name;
            }

            if (input_name_.empty() || keypoints_name_.empty())
                throw std::runtime_error("TensorRT engine must have one input and a keypoints output");

            ROS_INFO("Loaded SuperPoint TensorRT engine: %s", config.engine_path.c_str());
            ROS_INFO("SuperPoint input tensor: %s, keypoints: %s, scores: %s",
                     input_name_.c_str(), keypoints_name_.c_str(), scores_name_.c_str());
            return true;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Failed to initialize TensorRT SuperPoint backend: %s", e.what());
            return false;
        }
    }

    bool infer(const cv::Mat& image, std::vector<ScoredPoint>& points) override
    {
        points.clear();
        if (!engine_ || !context_ || image.empty() || image.type() != CV_8UC1)
            return false;

        try
        {
            cv::Mat float_image;
            image.convertTo(float_image, CV_32F, 1.0 / 255.0);
            if (!float_image.isContinuous())
                float_image = float_image.clone();

            nvinfer1::Dims input_dims{4, {1, 1, float_image.rows, float_image.cols}};
            nvinfer1::Dims declared_input_dims = engine_->getTensorShape(input_name_.c_str());
            if (hasDynamicDim(declared_input_dims))
            {
                if (!context_->setInputShape(input_name_.c_str(), input_dims))
                    throw std::runtime_error("failed to set TensorRT SuperPoint input shape");
            }
            else if (!sameDims(declared_input_dims, input_dims))
            {
                throw std::runtime_error("TensorRT SuperPoint engine input shape does not match current image");
            }

            std::vector<TensorBuffer> tensors;
            tensors.reserve(engine_->getNbIOTensors());
            std::vector<DynamicOutputAllocator*> dynamic_allocators;

            int input_index = -1;
            for (int i = 0; i < engine_->getNbIOTensors(); ++i)
            {
                TensorBuffer tensor;
                tensor.name = engine_->getIOTensorName(i);
                tensor.is_input = engine_->getTensorIOMode(tensor.name.c_str()) == nvinfer1::TensorIOMode::kINPUT;
                tensor.dtype = engine_->getTensorDataType(tensor.name.c_str());
                tensor.dims = tensor.is_input ? context_->getTensorShape(tensor.name.c_str())
                                              : context_->getTensorShape(tensor.name.c_str());

                if (tensor.is_input)
                    input_index = static_cast<int>(tensors.size());

                if (!tensor.is_input && hasDynamicDim(tensor.dims))
                {
                    tensor.dynamic_output = true;
                    tensor.allocator = new DynamicOutputAllocator();
                    dynamic_allocators.push_back(tensor.allocator);
                    if (!context_->setOutputAllocator(tensor.name.c_str(), tensor.allocator))
                        throw std::runtime_error("failed to set TensorRT dynamic output allocator");
                }
                else
                {
                    int64_t count = volume(tensor.dims);
                    size_t bytes = static_cast<size_t>(count) * dtypeSize(tensor.dtype);
                    if (!tensor.is_input)
                        tensor.host.resize(bytes);
                    checkCuda(cudaMalloc(&tensor.device, bytes), "cudaMalloc");
                    if (!context_->setTensorAddress(tensor.name.c_str(), tensor.device))
                        throw std::runtime_error("failed to set TensorRT tensor address");
                }

                tensors.push_back(tensor);
            }

            if (input_index < 0)
                throw std::runtime_error("no TensorRT input tensor found");

            cudaStream_t stream = nullptr;
            checkCuda(cudaStreamCreate(&stream), "cudaStreamCreate");

            std::vector<float> input_host(static_cast<size_t>(float_image.total()));
            std::memcpy(input_host.data(), float_image.ptr<float>(), input_host.size() * sizeof(float));
            TensorBuffer& input = tensors[static_cast<size_t>(input_index)];
            checkCuda(cudaMemcpyAsync(input.device, input_host.data(), input_host.size() * sizeof(float),
                                      cudaMemcpyHostToDevice, stream),
                      "cudaMemcpyAsync H2D");

            if (!context_->enqueueV3(stream))
                throw std::runtime_error("TensorRT SuperPoint enqueueV3 failed");

            for (size_t i = 0; i < tensors.size(); ++i)
            {
                TensorBuffer& tensor = tensors[i];
                if (tensor.is_input)
                    continue;

                if (tensor.dynamic_output)
                {
                    if (!tensor.allocator || !tensor.allocator->hasShape() || !tensor.allocator->device())
                        throw std::runtime_error("TensorRT dynamic output allocator did not receive output");
                    tensor.dims = tensor.allocator->dims();
                    tensor.device = tensor.allocator->device();
                    tensor.host.resize(static_cast<size_t>(volume(tensor.dims)) * dtypeSize(tensor.dtype));
                }

                checkCuda(cudaMemcpyAsync(tensor.host.data(), tensor.device, tensor.host.size(),
                                          cudaMemcpyDeviceToHost, stream),
                          "cudaMemcpyAsync D2H");
            }

            checkCuda(cudaStreamSynchronize(stream), "cudaStreamSynchronize");
            cudaStreamDestroy(stream);

            TensorBuffer* keypoints = nullptr;
            TensorBuffer* scores = nullptr;
            for (size_t i = 0; i < tensors.size(); ++i)
            {
                if (tensors[i].name == keypoints_name_)
                    keypoints = &tensors[i];
                else if (tensors[i].name == scores_name_)
                    scores = &tensors[i];
            }

            parseOutputs(*keypoints, scores, image.cols, image.rows, points);

            for (size_t i = 0; i < tensors.size(); ++i)
            {
                if (tensors[i].device && !tensors[i].dynamic_output)
                    cudaFree(tensors[i].device);
            }
            for (size_t i = 0; i < dynamic_allocators.size(); ++i)
                delete dynamic_allocators[i];
            return true;
        }
        catch (const std::exception& e)
        {
            ROS_ERROR_THROTTLE(1.0, "TensorRT SuperPoint inference failed: %s", e.what());
            return false;
        }
    }

private:
    static void parseOutputs(const TensorBuffer& keypoints,
                             const TensorBuffer* scores,
                             int width,
                             int height,
                             std::vector<ScoredPoint>& points)
    {
        points.clear();
        if (keypoints.dims.nbDims < 2 || keypoints.dims.d[keypoints.dims.nbDims - 1] != 2)
            throw std::runtime_error("SuperPoint keypoints output must end with dimension 2");

        const void* keypoint_data = keypoints.host.data();
        const void* score_data = scores ? scores->host.data() : nullptr;
        int64_t n = volume(keypoints.dims) / 2;
        int64_t score_count = scores ? volume(scores->dims) : 0;

        std::vector<cv::Point2f> xy;
        xy.reserve(static_cast<size_t>(n));
        for (int64_t i = 0; i < n; ++i)
        {
            float x = static_cast<float>(readAsDouble(keypoint_data, keypoints.dtype, i * 2));
            float y = static_cast<float>(readAsDouble(keypoint_data, keypoints.dtype, i * 2 + 1));
            xy.push_back(cv::Point2f(x, y));
        }

        std::vector<cv::Point2f> swapped = xy;
        for (size_t i = 0; i < swapped.size(); ++i)
            std::swap(swapped[i].x, swapped[i].y);

        int xy_valid = countInBounds(xy, width, height);
        int swapped_valid = countInBounds(swapped, width, height);
        const std::vector<cv::Point2f>& pts = swapped_valid > xy_valid ? swapped : xy;

        points.reserve(static_cast<size_t>(n));
        for (int64_t i = 0; i < n; ++i)
        {
            const cv::Point2f& p = pts[static_cast<size_t>(i)];
            if (p.x < 0 || p.x >= width || p.y < 0 || p.y >= height)
                continue;

            float score = 1.0f;
            if (score_data && i < score_count)
                score = static_cast<float>(readAsDouble(score_data, scores->dtype, i));
            points.push_back(ScoredPoint{p, score});
        }
    }

    static int countInBounds(const std::vector<cv::Point2f>& pts, int width, int height)
    {
        int count = 0;
        for (size_t i = 0; i < pts.size(); ++i)
        {
            if (pts[i].x >= 0 && pts[i].x < width && pts[i].y >= 0 && pts[i].y < height)
                ++count;
        }
        return count;
    }

    std::unique_ptr<nvinfer1::IRuntime> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext> context_;
    std::string input_name_;
    std::string keypoints_name_;
    std::string scores_name_;
};

}  // namespace

class TensorRtDetector::Impl
{
public:
    bool init(const SuperPointConfig& config)
    {
        config_ = config;
        if (!config_.enable)
            return false;

        if (config_.backend != "tensorrt" && config_.backend != "trt")
        {
            ROS_ERROR("Unsupported SuperPoint backend '%s'. Only TensorRT is implemented now.", config_.backend.c_str());
            return false;
        }

        backend_.reset(new TrtSuperPointBackend());
        initialized_ = backend_->init(config_);
        return initialized_;
    }

    bool isInitialized() const { return initialized_; }

    bool detect(const cv::Mat& image,
                const cv::Mat& mask,
                int max_new_points,
                int min_dist,
                std::vector<cv::Point2f>& points)
    {
        points.clear();
        if (!initialized_ || !backend_ || max_new_points <= 0)
            return false;

        std::vector<ScoredPoint> candidates;
        if (!backend_->infer(image, candidates))
            return false;

        std::sort(candidates.begin(), candidates.end(), [](const ScoredPoint& a, const ScoredPoint& b) {
            return a.score > b.score;
        });

        cv::Mat work_mask = mask.empty() ? cv::Mat(image.rows, image.cols, CV_8UC1, cv::Scalar(255)) : mask.clone();
        const int border = std::max(0, config_.border);
        int accepted = 0;
        int visited = 0;
        for (size_t i = 0; i < candidates.size() && accepted < max_new_points; ++i)
        {
            const ScoredPoint& candidate = candidates[i];
            if (config_.max_keypoints > 0 && visited++ >= config_.max_keypoints)
                break;
            if (candidate.score < config_.score_threshold)
                continue;

            int x = cvRound(candidate.pt.x);
            int y = cvRound(candidate.pt.y);
            if (x < border || x >= image.cols - border || y < border || y >= image.rows - border)
                continue;
            if (work_mask.at<uchar>(y, x) == 0)
                continue;

            points.push_back(candidate.pt);
            ++accepted;
            cv::circle(work_mask, candidate.pt, min_dist, cv::Scalar(0), -1);
        }

        return true;
    }

private:
    SuperPointConfig config_;
    bool initialized_ = false;
    std::unique_ptr<SuperPointBackend> backend_;
};

TensorRtDetector::TensorRtDetector() : impl_(new Impl()) {}

TensorRtDetector::~TensorRtDetector() {}

bool TensorRtDetector::init(const SuperPointConfig& config)
{
    return impl_->init(config);
}

bool TensorRtDetector::isInitialized() const
{
    return impl_->isInitialized();
}

bool TensorRtDetector::detect(const cv::Mat& image,
                                 const cv::Mat& mask,
                                 int max_new_points,
                                 int min_dist,
                                 std::vector<cv::Point2f>& points)
{
    return impl_->detect(image, mask, max_new_points, min_dist, points);
}
