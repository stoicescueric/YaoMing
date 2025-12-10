package org.firstinspires.ftc.teamcode.Util.Wrapper;

import com.acmerobotics.dashboard.config.Config;

import java.util.ArrayDeque;
import java.util.Deque;

@Config
public class BinaryDeque {

    private final Deque<Integer> deque;
    private int currentSum;
    public static  int MAX_SIZE = 20;
    public static int THRESHOLD = 15;
    public BinaryDeque() {
        this.deque = new ArrayDeque<>(MAX_SIZE);
        this.currentSum = 0;

        for (int i = 0; i < MAX_SIZE; i++) {
            deque.addLast(0);
        }
    }


    public void slide(int value) {


        int removedElement = deque.removeFirst();

        currentSum -= removedElement;

        deque.addLast(value);

        currentSum += value;
    }


    public int getSum() {
        return currentSum;
    }

    public boolean isAboveThreshold() {
        return currentSum >= THRESHOLD;
    }



}









