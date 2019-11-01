package org.aceshigh176.lib.externalactions;

import java.io.Serializable;
import java.util.function.Consumer;

public interface ArbitraryCodeExecutorLambda extends Consumer<Void>, Serializable {
    void accept(Void t);
}
