package frc.robot.utilities;

import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.nio.ByteBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PiecePickerPoseProvider {
    private DatagramSocket socket;
    private byte buffer[] = new byte[1024];
    PieceEstimatedPose estimatedPose;

    public PiecePickerPoseProvider() throws Exception {
        socket = new DatagramSocket(4445);

    }

    public static void main(String argp[]) {
        byte buffer[]= new byte[32];
        for (byte i=0;i<buffer.length;++i) {
            buffer[i] =(byte) (7 -(i % 8));
        }
        for (int i=0;i<buffer.length;++i) {
            System.out.println(buffer[i]);
        }
        double timestamp = ByteBuffer.wrap(buffer,0,8).getDouble();
        double x = ByteBuffer.wrap(buffer,8,8).getDouble();
        double y = ByteBuffer.wrap(buffer,16,8).getDouble();
        double angle = ByteBuffer.wrap(buffer,24,8).getDouble();        
        Pose2d pose = new Pose2d(x,y,new Rotation2d(angle));
        System.out.println("timestamp: "+timestamp);
        System.out.println("x = "+x);
        System.out.println("y = "+y);
        System.out.println("angle = "+angle);
        System.out.println("pose: "+pose);
    }

    public void run() throws Exception {
        boolean running = true;
        while (running) {
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            socket.receive(packet);
            double timestamp = ByteBuffer.wrap(buffer,0,8).getDouble();
            double x = ByteBuffer.wrap(buffer,8,8).getDouble();
            double y = ByteBuffer.wrap(buffer,16,8).getDouble();
            double angle = ByteBuffer.wrap(buffer,24,8).getDouble();      
            Pose2d pose = new Pose2d(x,y,new Rotation2d(angle));
            estimatedPose = new PieceEstimatedPose(pose, timestamp);
        }
        socket.close();
    }

    public boolean hasResult() {
        return estimatedPose != null;
    }

    public PieceEstimatedPose getResult() {
        return estimatedPose;
    }

    public class PieceEstimatedPose {
        Pose2d pose;
        double timestamp;
        public PieceEstimatedPose(Pose2d pose, double timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }

        public Pose2d getPose() {
            return pose;
        }

        public double getTimestamp() {
            return timestamp;
        }
    }

}
