package kinectviewerapp;

import java.awt.Color;
import java.awt.Transparency;
import java.awt.color.ColorSpace;
import java.awt.image.BufferedImage;
import java.awt.image.ColorModel;
import java.awt.image.ComponentColorModel;
import java.awt.image.DataBuffer;
import java.awt.image.DataBufferByte;
import java.awt.image.Raster;
import java.awt.image.RenderedImage;
import java.awt.image.WritableRaster;
import java.io.ByteArrayInputStream;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.JLabel;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import edu.ufl.digitalworlds.j4k.DepthMap;
import edu.ufl.digitalworlds.j4k.J4KSDK;
import edu.ufl.digitalworlds.j4k.Skeleton;

/*
 * Copyright 2011-2014, Digital Worlds Institute, University of 
 * Florida, Angelos Barmpoutis.
 * All rights reserved.
 *
 * When this program is used for academic or research purposes, 
 * please cite the following article that introduced this Java library: 
 * 
 * A. Barmpoutis. "Tensor Body: Real-time Reconstruction of the Human Body 
 * and Avatar Synthesis from RGB-D', IEEE Transactions on Cybernetics, 
 * October 2013, Vol. 43(5), Pages: 1347-1356. 
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain this copyright
 * notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce this
 * copyright notice, this list of conditions and the following disclaimer
 * in the documentation and/or other materials provided with the
 * distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
public class Kinect extends J4KSDK {

	float upperThreshold;
	float lowerThreshold;
	ViewerPanel3D viewer = null;
	JLabel label = null;
	boolean mask_players = false;
	int i = 0;
	private static String windowsOpencvPath = System.getProperty("user.dir") + "\\lib\\" + Core.NATIVE_LIBRARY_NAME
			+ ".dll";

	public void maskPlayers(boolean flag) {
		mask_players = flag;
	}

	public Kinect() {
		super();
		System.load(windowsOpencvPath);
		System.out.println("Opencv loaded");
	}

	public Kinect(byte type) {
		super(type);
	}

	public void setViewer(ViewerPanel3D viewer) {
		this.viewer = viewer;
	}

	public void setLabel(JLabel l) {
		this.label = l;
	}

	private boolean use_infrared = false;

	public void updateTextureUsingInfrared(boolean flag) {
		use_infrared = flag;
	}

	@Override
	public void onDepthFrameEvent(short[] depth_frame, byte[] player_index, float[] XYZ, float[] UV) {

		if (viewer == null || label == null)
			return;
		float a[] = getAccelerometerReading();
		label.setText(
				((int) (a[0] * 100) / 100f) + "," + ((int) (a[1] * 100) / 100f) + "," + ((int) (a[2] * 100) / 100f));
		DepthMap map = new DepthMap(getDepthWidth(), getDepthHeight(), XYZ);

		map.maskZ(upperThreshold, lowerThreshold);
		map.setMaximumAllowedDeltaZ(0.5);

//		short[] outputAsShort = map.getImageWithMaskedZ(upperThreshold, lowerThreshold);
		byte[] outputAsByte = getImageWithMaskedZ(upperThreshold, lowerThreshold, map);
//		byte[] outputAsByte = shortToByteTwiddlMethod(outputAsShort);
		Mat mat = new Mat(getDepthHeight(), getDepthWidth(), CvType.CV_8UC1);
		Core.flip(mat, mat, 1);
		Imgcodecs.imencode(".jpg", mat, new MatOfByte(outputAsByte));
		Imgcodecs.imwrite("dupa.jpg", mat);
		
//		findCircles(mat);

		// mat.put(0, 0, outputAsByte);
//		System.out.println(mat.dump());

//		Mat mat = Imgcodecs.imdecode(new MatOfByte(outputAsByte), Imgcodecs.CV_LOAD_IMAGE_UNCHANGED);
//		System.out.println(new MatOfByte(outputAsByte));

//		for (int j = 0 ; j < outputAsByte.length ; j ++) {
//			System.out.println(Character.getNumericValue((char) outputAsByte[j]));
//		}

		try (FileOutputStream fos = new FileOutputStream("pliczek")) {
			fos.write(outputAsByte);
		} catch (FileNotFoundException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

//		try {
//			// retrieve image
//			BufferedImage writeImage = new BufferedImage(getDepthWidth(), getDepthHeight(), BufferedImage.TYPE_BYTE_GRAY);
//			File outputfile = new File("saved" + i + ".png");
//			WritableRaster raster = (WritableRaster) writeImage.getData();
//			raster.setPixels(0, 0, getDepthWidth(), getDepthHeight(), outputAsInt);
//			writeImage.setData(raster);
//			i ++;
//			
//			ImageIO.write(writeImage, "png", outputfile);
//
//		} catch (IOException e) {
//
//		}

//		BufferedImage image;
//		try {
//			image = ImageIO.read( new ByteArrayInputStream( outputAsByte ) );
//			ImageIO.write(image, "BMP", new File("filename" + i + ".bmp"));
//			i ++;
//		} catch (IOException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}

		if (UV != null && !use_infrared)
			map.setUV(UV);
		else if (use_infrared)
			map.setUVuniform();
		if (mask_players) {
			map.setPlayerIndex(depth_frame, player_index);
			map.maskPlayers();
		}
		viewer.map = map;
	}
	

	public byte[] getImageWithMaskedZ(float thresholdUp, float thresholdDown, DepthMap map) {
		int sz = map.getWidth() * map.getHeight();
		byte[] output = new byte[sz];

		for (int i = 0; i < sz; i++) {
			if (map.realZ[i] < thresholdUp && map.realZ[i] > thresholdDown) {
//				output[i] = (byte) rescaleValue(map.realZ[i]);
				output[i] = (byte) 255;
			} else {
				output[i] = 0;
			}
		}
		

		return output;
	}
	
	public void drawBall(Mat img, Ball ball, Scalar color) {
		Point center = new Point(ball.getX(), ball.getY());
		Imgproc.circle(img, center, 4, color, 3);
	}
	
	public List<Ball> detectBalls(Mat sourceImg) {
		Mat blurredImage = new Mat();
		Mat convertedTypeImage = new Mat();
		Mat destinationMat = new Mat();
		Size blurSize = new Size(2, 2);
		
		Imgproc.blur(sourceImg, blurredImage, blurSize);
		Imgproc.cvtColor(blurredImage, convertedTypeImage, Imgproc.COLOR_BGR2HSV);

		// split into planes
		List<Mat> planes = new ArrayList<>(3);
		Core.split(convertedTypeImage, planes);
		convertedTypeImage.release();
		
		int thresh = 240;
		
		Mat cannyImg = new Mat();
		Imgproc.Canny(planes.get(2), cannyImg, thresh / 3, thresh);
		Imgcodecs.imwrite("canny.png", cannyImg);

		
		
		// detect circles
		Imgproc.HoughCircles(planes.get(2), destinationMat, Imgproc.CV_HOUGH_GRADIENT, 1.0,
				6, 225, 6.5, 1, 6);
		
		List<Ball> ballList = convertMatToListOfBalls(destinationMat);
		System.out.println(ballList);
		
		return ballList;
	}
	
	private List<Ball> convertMatToListOfBalls(Mat circles) {
		int x;
		int y;
		int r;
		List<Ball> balls = new ArrayList<>();

		for (int i = 0; i < circles.cols(); i++) {
			// read ball coordinates
			double[] data = circles.get(0, i);

			x = (int) data[0];
			y = (int) data[1];
			r = (int) data[2];

			Ball ball = new Ball(x, y, r);
			balls.add(ball);
		}

		return balls;
	}
	
	public float rescaleValue(float value) {
		float minRescale = 0;
		float maxRescale = 255;
		float minThreshold = lowerThreshold;
		float maxThreshold = upperThreshold;

		return 255 -  (minRescale + (((value - minThreshold) / (maxThreshold - minThreshold)) * (maxRescale - minRescale)));
		}

	public void findCircles(Mat gray) {

		Mat circles = new Mat();

		Imgproc.HoughCircles(gray, circles, Imgproc.HOUGH_GRADIENT, 1.0, (double) gray.rows() / 16, 100.0, 5, 1, 500);
		
		System.out.println(circles.dims());
		for (int x = 0; x < circles.cols(); x++) {
			double[] c = circles.get(0, x);
			Point center = new Point(Math.round(c[0]), Math.round(c[1]));
			// circle center
			Imgproc.circle(gray, center, 1, new Scalar(0, 100, 100), 3, 8, 0);
			// circle outline
			int radius = (int) Math.round(c[2]);
			Imgproc.circle(gray, center, radius, new Scalar(255, 0, 255), 3, 8, 0);
		}
		
		Imgcodecs.imwrite("after.jpg", gray);
	}

	byte[] shortToByteTwiddlMethod(short[] input) {
		byte[] buffer = new byte[input.length];

		for (int i = 0; i < input.length; i++) {
			buffer[i] = (byte) (input[i]);
			System.out.println("SHORT: " + input[i]);
			System.out.println("BYTE: " + buffer[i]);
//			if (buffer[i] > 0 ) {
//				buffer[i] = (byte) 255;
//			}
		}

		return buffer;
	}

	@Override
	public void onSkeletonFrameEvent(boolean[] flags, float[] positions, float[] orientations, byte[] state) {
		if (viewer == null || viewer.skeletons == null)
			return;

		for (int i = 0; i < getSkeletonCountLimit(); i++) {
			viewer.skeletons[i] = Skeleton.getSkeleton(i, flags, positions, orientations, state, this);
		}
	}

	public float getUpperThreshold() {
		return upperThreshold;
	}

	public void setUpperThreshold(float upperThreshold) {
		this.upperThreshold = upperThreshold;
	}

	public float getLowerThreshold() {
		return lowerThreshold;
	}

	public void setLowerThreshold(float lowerThreshold) {
		this.lowerThreshold = lowerThreshold;
	}

	@Override
	public void onColorFrameEvent(byte[] data) {
		
		Mat mat = new Mat(getColorHeight(), getColorWidth(), CvType.CV_8UC4);
		mat.put(0, 0, data);
		Core.flip(mat, mat, 1);
		Imgcodecs.imwrite("color.jpg", mat);

		if (viewer == null || viewer.videoTexture == null || use_infrared)
			return;
		viewer.videoTexture.update(getColorWidth(), getColorHeight(), data);
	}

	@Override
	public void onInfraredFrameEvent(short[] data) {
		if (viewer == null || viewer.videoTexture == null || !use_infrared)
			return;
		int sz = getInfraredWidth() * getInfraredHeight();
		byte bgra[] = new byte[sz * 4];
		int idx = 0;
		int iv = 0;
		short sv = 0;
		byte bv = 0;
		for (int i = 0; i < sz; i++) {
			sv = data[i];
			iv = sv >= 0 ? sv : 0x10000 + sv;
			bv = (byte) ((iv & 0xfff8) >> 6);
			bgra[idx] = bv;
			idx++;
			bgra[idx] = bv;
			idx++;
			bgra[idx] = bv;
			idx++;
			bgra[idx] = 0;
			idx++;
		}

		viewer.videoTexture.update(getInfraredWidth(), getInfraredHeight(), bgra);
	}

}
