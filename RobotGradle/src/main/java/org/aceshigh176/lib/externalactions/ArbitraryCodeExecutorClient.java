package org.aceshigh176.lib.externalactions;

import org.aceshigh176.lib.loops.Loopable;
import org.aceshigh176.lib.loops.Looper;
import org.aceshigh176.lib.loops.ThreadedLooper;
import org.aceshigh176.lib.util.UnitUtil;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.ObjectOutputStream;
import java.net.Socket;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ArbitraryCodeExecutorClient implements Loopable {

//    private final Logger log = LogManager.getLogger(ArbitraryCodeExecutorClient.class);

    private final Looper mLooper = new ThreadedLooper(UnitUtil.hzToPeriod(100), "ArbitraryCodeExecutorClient", Thread.NORM_PRIORITY);
    private final ConcurrentLinkedQueue<ArbitraryCodeExecutorLambda> mQueue = new ConcurrentLinkedQueue<>();
    private State mState = State.CONNNECT;
    private Socket socket;
    private ObjectOutputStream oos;
    private BufferedReader reader;

    public ArbitraryCodeExecutorClient() {
        mLooper.register(this);
    }

    public void submitForExecution(ArbitraryCodeExecutorLambda function) {
        mQueue.add(function);
    }

    @Override
    public void loop(double timestamp) {
        State nextState = mState;

        if(mState == State.CONNNECT) {
            try {
//                log.debug("Connecting to server");
				System.out.println("Connecting to server");
                socket = new Socket("10.1.95.19", 9010);
//                log.debug("Connected to server");
                oos = new ObjectOutputStream(socket.getOutputStream());
                reader = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                nextState = State.SEND;
            } catch (IOException e) {
            	e.printStackTrace();
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e1) {
                }
            }
        } else if(mState == State.SEND) {
            try {
                while(!mQueue.isEmpty()) {
//                    log.debug("mQueue is not empty");
                    oos.writeObject(mQueue.peek());
                    oos.flush();
                    reader.readLine();
//                    log.debug(reader.readLine());
                    mQueue.poll();
                }
            } catch (IOException e) {
                e.printStackTrace();
                try {
                    socket.close();
                } catch (IOException e1) {
                    e1.printStackTrace();
                }
                nextState = State.CONNNECT;
            }
        }

        mState = nextState;
    }

    public enum State {
        CONNNECT,
        SEND
    }
}
