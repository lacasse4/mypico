#include <stdio.h>

// Function to calculate the linear least square estimation
void calculateLinearLeastSquare(double x[], double y[], int n, double *slope, double *intercept) {
    double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
    for (int i = 0; i < n; i++) {
        sumX += x[i];
        sumY += y[i];
        sumXY += x[i] * y[i];
        sumX2 += x[i] * x[i];
    }
    *slope = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);
    *intercept = (sumY - (*slope) * sumX) / n;
}

// Function to calculate y using the linear least square estimation
double calculateY(double slope, double intercept, double x) {
    return slope * x + intercept;
}

int main() {
    double x[5] = {1, 2, 3, 4, 5};
    double y[5] = {2.2, 2.8, 3.6, 4.5, 5.1};
    double slope, intercept;

    // Calculate the linear least square estimation
    calculateLinearLeastSquare(x, y, 5, &slope, &intercept);

    printf("Linear Least Square Estimation:\n");
    printf("Slope: %f\n", slope);
    printf("Intercept: %f\n", intercept);

    // Calculate y for a new x value
    double newX = 6;
    double newY = calculateY(slope, intercept, newX);
    printf("For x = %f, estimated y = %f\n", newX, newY);

    return 0;
}