package kinectviewerapp;

import org.opencv.core.Point;

import javax.xml.bind.annotation.*;

/**
 * Ball class
 */

@XmlType(name="ball")
@XmlAccessorType(XmlAccessType.FIELD)
public class Ball implements Comparable<Ball> {
    @XmlElement(required = true)
    private int id;

    public static final int DEFAULT_ID = 1;
    @XmlTransient
    private double radius;

    @XmlElement(required = true)
    private double x;

    @XmlElement(required = true)
    private double y;

    @XmlTransient
    private double G;

    @XmlTransient
    private double B;

    @XmlTransient
    private double R;

    @XmlTransient
    private int tag;

    @XmlTransient
    private int isInStaticCounter;

    @XmlTransient
    private double whitePixels;

    @Override
    public int compareTo(Ball b) {
        if(this.id < b.id) {
            return -1;
        } else if(b.id < this.id) {
            return 1;
        }else {
            return 0;
        }
    }

    @Override
    public String toString() {
        return "Ball{" +
                "id=" + id +
                ", radius=" + radius +
                ", x=" + x +
                ", y=" + y +
                ", white=" + whitePixels +
                '}';
    }

    public double getWhitePixels() {
        return whitePixels;
    }

    public void setWhitePixels(double whitePixels) {
        this.whitePixels = whitePixels;
    }

    public double getR() {
        return R;
    }

    public void setR(double r) {
        R = r;
    }

    public double getG() {
        return G;
    }

    public void setG(double g) {
        G = g;
    }

    public double getB() {
        return B;
    }

    public void setB(double b) {
        B = b;
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }



    /**
     * Ball Constructor
     *
     * @param x  position X
     * @param y  position Y
     */
    public Ball(double x, double y, double radius) {
        this.id = DEFAULT_ID;
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.isInStaticCounter = 0;
    }

    public Ball(double x, double y, double radius, int id) {
        this.id = id;
        this.x = x;
        this.y = y;
        this.radius = radius;
        this.isInStaticCounter = 0;
    }    
    
    /**
     * Default Ball constructor
     */
    public Ball() {}

    /**
     * Get Ball id
     *
     * @return Ball id
     */
    public Integer getId() {
        return id;
    }

    /**
     * Set Ball id
     *
     * @param id Ball id
     */
    public void setId(Integer id) {
        this.id = id;
    }

    /**
     * Get Ball x position
     *
     * @return Ball x position
     */
    public double getX() {
        return x;
    }

    /**
     * Set Ball x position
     *
     * @param x Ball x position
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Get Ball y position
     *
     * @return Ball y position
     */
    public double getY() {
        return y;
    }

    /**
     * Set Ball y position
     *
     * @param y Ball y position
     */
    public void setY(double y) {
        this.y = y;
    }

    @Override
    public boolean equals(Object o) {
        if (!(o instanceof Ball)) {
            return false;
        }

        Ball b = (Ball) o;

        double xDelta = Math.abs(this.x - b.getX());
        double yDelta = Math.abs(this.y - b.getY());

        return (xDelta <= 5.0 && yDelta <= 5.0);
    }

    public Point getCenter() {
        return new Point(x, y);
    }

    public void setCenter(Point point) {
        this.x = point.x;
        this.y = point.y;
    }

    public final int getStaticCounter(){
        return isInStaticCounter;
    }

    public void increaseStaticCounter(){
        this.isInStaticCounter++;
    }

	public int getTag() {
		return tag;
	}

	public void setTag(int tag) {
		this.tag = tag;
	}
}
