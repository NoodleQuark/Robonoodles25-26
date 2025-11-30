package org.firstinspires.ftc.teamcode;

public class Stopwatch {

    private long startTime = 0;
    private long pauseStart = 0;
    private long pausedTotal = 0;

    public boolean running = false;
    public boolean paused = false;


    public void start(){
        startTime = System.currentTimeMillis();
        pausedTotal = 0;
        paused = false;
        running = true;
    }
    public void pause(){
        if(running && !paused){
            pauseStart = System.currentTimeMillis();
            paused = true;
        }
    }
    public void end(){
        running = false;
        paused = false;
        pausedTotal = 0;
        startTime = 0;
    }

    public void resume(){
        if(paused){
            pausedTotal += System.currentTimeMillis() - pauseStart;
            paused = false;
        }
    }
    public long elapsed(){
        if(!running) return 0;

        if(paused)return (pauseStart - startTime) - pausedTotal;

        return (System.currentTimeMillis() - startTime) - pausedTotal;
    }

}
