
namespace webots
{
    class GPS : public Device
    {
    public:
        typedef enum
        {
            LOCAL = 0,
            WGS84
        } CoordinateSystem;
        explicit GPS(const std::string &name) : Device(name) {}
        // Use Robot::getGPS() instead
        virtual ~GPS() {}
        // Functions:
        virtual void enable(int samplingPeriod);
        virtual void disable();
        int getSamplingPeriod() const;
        const double *getValues() const;
    }
}
