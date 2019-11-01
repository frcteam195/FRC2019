package org.aceshigh176.lib.externalactions;

import org.aceshigh176.lib.loops.Loopable;
import org.aceshigh176.lib.loops.Looper;
import org.aceshigh176.lib.loops.ThreadedLooper;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

public class ArbitraryCodeExecutorServer implements Loopable {


    private final Looper looper = new ThreadedLooper(.1, "ArbitraryCodeExecutorServer", Thread.NORM_PRIORITY);

    private State mState = State.INIT;
    private ServerSocket serverSocket;

    public ArbitraryCodeExecutorServer() {
        looper.register(this);
    }

    @Override
    public void loop(double timestamp) {
        State nextState = mState;

        if(mState == State.INIT) {
            try {
                System.out.println("created server socket");
                serverSocket = new ServerSocket(9010);
//                System.out.println("server socket created");
                nextState = State.LISTEN;
            } catch (IOException e) {
                System.out.println(e);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e1) {
                    System.out.println(e);
                }
            }
        } else if(mState == State.LISTEN) {
            try {
                System.out.println("Listening for client");
                Socket client = serverSocket.accept();
                System.out.println("Client connected");
                Runnable clientThread = new Runnable() {
                    @Override
                    public void run() {
                        Thread.currentThread().setName("ArbitraryCodeExecutionClient " + client.getInetAddress().getHostAddress());
                        try {
                            InputStream is = client.getInputStream();
                            OutputStream os = client.getOutputStream();

                            ObjectInputStream ois = new ObjectInputStream(is);
                            BufferedWriter bufferedWriter = new BufferedWriter(new OutputStreamWriter(os));
//                            System.out.println("just before while loop");
                            while(client.isConnected()) {
//                                System.out.println("reading object");
                                ArbitraryCodeExecutorLambda function = (ArbitraryCodeExecutorLambda) ois.readObject();
//                                System.out.println("got object object");
                                function.accept(null);
//                                System.out.println("accepted that object");
                                bufferedWriter.write("Got it!\n");
                                bufferedWriter.flush();
                            }
                        } catch(IOException | ClassNotFoundException e) {
                            System.out.println(e);
                        } finally {
                            try {
                                client.close();
                            } catch (IOException e) {
                                System.out.println(e);
                            }
                        }
                    }
                };
                new Thread(clientThread).start();
            } catch (IOException e) {
                System.out.println(e);
            }
        } else if(mState == State.CLOSE) {

        }

        mState = nextState;
    }

    private enum State {
        INIT,
        LISTEN,
        CLOSE,
    }
}
