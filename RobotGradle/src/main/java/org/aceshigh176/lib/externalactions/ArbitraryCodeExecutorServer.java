package org.aceshigh176.lib.externalactions;

import org.aceshigh176.lib.loops.Loopable;
import org.aceshigh176.lib.loops.Looper;
import org.aceshigh176.lib.loops.ThreadedLooper;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;

public class ArbitraryCodeExecutorServer implements Loopable {

    private final Logger log = LogManager.getLogger(ArbitraryCodeExecutorServer.class);

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
                log.debug("created server socket");
                serverSocket = new ServerSocket(9010);
//                log.debug("server socket created");
                nextState = State.LISTEN;
            } catch (IOException e) {
                log.error(e);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e1) {
                    log.error(e);
                }
            }
        } else if(mState == State.LISTEN) {
            try {
                log.debug("Listening for client");
                Socket client = serverSocket.accept();
                log.debug("Client connected");
                Runnable clientThread = new Runnable() {
                    @Override
                    public void run() {
                        Thread.currentThread().setName("ArbitraryCodeExecutionClient " + client.getInetAddress().getHostAddress());
                        try {
                            InputStream is = client.getInputStream();
                            OutputStream os = client.getOutputStream();

                            ObjectInputStream ois = new ObjectInputStream(is);
                            BufferedWriter bufferedWriter = new BufferedWriter(new OutputStreamWriter(os));
//                            log.debug("just before while loop");
                            while(client.isConnected()) {
//                                log.debug("reading object");
                                ArbitraryCodeExecutorLambda function = (ArbitraryCodeExecutorLambda) ois.readObject();
//                                log.debug("got object object");
                                function.accept(null);
//                                log.debug("accepted that object");
                                bufferedWriter.write("Got it!\n");
                                bufferedWriter.flush();
                            }
                        } catch(IOException | ClassNotFoundException e) {
                            log.error(e);
                        } finally {
                            try {
                                client.close();
                            } catch (IOException e) {
                                log.error(e);
                            }
                        }
                    }
                };
                new Thread(clientThread).start();
            } catch (IOException e) {
                log.error(e);
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
