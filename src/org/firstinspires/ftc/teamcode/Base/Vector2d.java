package org.firstinspires.ftc.teamcode.Base;

public class Vector2d {
    public double x = 0.0;
    public double y = 0.0;

    public Vector2d() {}

    @SuppressWarnings("ParameterName")
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Rotate a vector in Cartesian space.
     *
     * @param angle angle in degrees by which to rotate vector counter-clockwise.
     */
    public void rotate(double angle) {
        double cosA = Math.cos(angle * (Math.PI / 180.0));
        double sinA = Math.sin(angle * (Math.PI / 180.0));
        double[] out = new double[2];
        out[0] = x * cosA - y * sinA;
        out[1] = x * sinA + y * cosA;
        x = out[0];
        y = out[1];
    }

    /**
     * Returns dot product of this vector with argument.
     *
     * @param vec Vector with which to perform dot product.
     */
    public double dot(Vector2d vec) {
        return x * vec.x + y * vec.y;
    }

    /**
     * Returns magnitude of vector.
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns scalar projection of this vector onto argument.
     *
     * @param vec Vector onto which to project this vector.
     */
    public double scalarProject(Vector2d vec) {
        return dot(vec) / vec.magnitude();
    }
}

