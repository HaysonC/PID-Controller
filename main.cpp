#include <iostream>
#include <fstream>
#include <vector>
#include <cstdlib>



// PID controller program

// derivative filtering and anti-windup mechanism
// a simulink view of the PID controller is provided along with the submission



// plant of the controller is to control the speed of a motor in an automotive

// PID would control the speed of the motor by adjusting the voltage to the motor

// TODO: IMPLEMENT PID CONTROLLER CLASS with derivative filtering and anti-windup mechanism
class PID {
private:
   // PID coefficients
   double kp;  // Proportional gain
   double ki;  // Integral gain
   double kd;  // Derivative gain


   // PID state variables
   double integral;
   double previous_error;
   double output_min;  // Minimum output limit
   double output_max;  // Maximum output limit


   // Optional: For derivative filtering
   double last_derivative;
   double alpha;  // Smoothing factor for derivative filter


public:
   // Constructor
   PID(double kp, double ki, double kd, double output_min, double output_max, double alpha)
       : kp(kp), ki(ki), kd(kd), integral(0.0), previous_error(0.0),
         output_min(output_min), output_max(output_max), last_derivative(0.0), alpha(alpha)  {
       // Initialize any other variables if necessary
   }


   // Method to calculate the control output
   double calculate(double setpoint, double measured_value, double dt) {
       // Error
       double error = setpoint - measured_value;

       // P term
       double Pout = kp * error;

       // I term
       integral += error * dt;
       double Iout = ki * integral;


       //self.derivative = (self.derivative_filter_coeff * raw_derivative +
       //(1 - self.derivative_filter_coeff) * self.previous_derivative)
       // D term with derivative filtering using the above python code
       double raw_derivative = (error - previous_error) / dt;
       double derivative = alpha * raw_derivative + (1 - alpha) * last_derivative;
       double Dout = kd * derivative;

       // update the previous error
       previous_error = error;


       // Clamp output to min and max limits
       double output = Pout + Iout + Dout;
        if (output > output_max) {
            output = output_max;
            integral -= error * dt;  // Prevent integral windup
        } else if (output < output_min) {
            output = output_min;
            integral -= error * dt;  // Prevent integral windup
        }
       std::cout << "Pout: " << Pout << " Iout: " << Iout << " Dout: " << Dout << std::endl;

       return output;
   }


   // Method to reset PID terms
   void reset() {
       integral = 0.0;
       previous_error = 0.0;
       last_derivative = 0.0;  // Reset if using derivative filtering

   }
};

class Plant {
private:
    double speed;  // Current speed of the motor in rpm
    double max_speed;
    double min_speed;
    double max_voltage;
    double min_voltage;
    double max_acceleration;
    double min_acceleration;
    double ratio;  // Ratio between voltage and speed (rpm/V)

public:
    Plant(double max_voltage, double min_voltage, double max_acceleration, double min_acceleration, double ratio)
        : speed(0.0), max_voltage(max_voltage), min_voltage(min_voltage), max_acceleration(max_acceleration), min_acceleration(min_acceleration), ratio(ratio) {}

    // Method to update the speed based on the voltage command
    void update(double voltage, double dt) {
        // Clamp the voltage to the allowed range
        voltage = std::clamp(voltage, min_voltage, max_voltage);

        // Calculate the desired speed change
        double desired_speed_change = voltage * ratio*dt;

        // Clamp the speed change to the allowed acceleration range
        double speed_change = std::clamp(desired_speed_change, min_acceleration * dt, max_acceleration * dt);

        // Update the speed
        speed += speed_change;

        // Clamp the speed to the allowed range
        speed = std::clamp(speed, min_voltage*ratio, max_voltage*ratio);
    }

    // Method to get the current speed
    double get_speed() const {
        return speed;
    }

    void add_speed(double speed) {
        this->speed += speed;
    }
};



int main() {
    // PID controller parameters
    double kp = 0.7, ki = 0.15, kd = 0.3;
    double alpha = 0.4;  // Smoothing factor for derivative filter
    double output_min = -14;
    double output_max = 14;

    // Create PID controller
    PID pid(kp, ki, kd, output_min, output_max, alpha);
    Plant plant(16.0, -16.0, 10.0, -10.0, 10.0);


    double setpoint = 70;
    double measured_value = 0;
    double dt = 0.01;  // Time step
    plant.update(0, dt);
    double time = 300;
    int tmax = int(time / dt);
    std::vector<double> output_array(tmax);

    for (int t = 0; t < tmax; t++) {
        measured_value = plant.get_speed();
        output_array[t] = measured_value;
        // add noise no greater than 5% of the measured value
        measured_value += measured_value * (rand() % 100 - 5) / 100.0;
        plant.update(pid.calculate(setpoint, measured_value, dt), dt);
        std::cout << "PID output: " << measured_value << std::endl;
    }

    // Write data to a file
    std::ofstream data_file("output_data.txt");
    for (int t = 0; t < tmax; t++) {
        data_file << t * dt << " " << output_array[t] << "\n";
    }
    data_file.close();

    // Use gnuplot to plot the data
    std::ofstream gnuplot_script("plot_script.gp");
    gnuplot_script << "set xlabel 'Time (s)'\n";
    gnuplot_script << "set ylabel 'Speed (rpm)'\n";
    gnuplot_script << "set title 'PID Controller Output'\n";
    gnuplot_script << "plot 'output_data.txt' with lines\n";
    // plot the setpoint as a horizontal line
    gnuplot_script.close();

    // Execute the gnuplot script
    std::system("gnuplot -p plot_script.gp");

    return 0;
}
