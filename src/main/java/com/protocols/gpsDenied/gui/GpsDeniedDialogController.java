package com.protocols.gpsDenied.gui;

import com.api.ArduSimTools;
import com.setup.Param;
import com.setup.Text;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Properties;
import java.util.ResourceBundle;

public class GpsDeniedDialogController {

    private final ResourceBundle         resources;
    private final GpsDeniedSimProperties properties;

    // fx:id names must match field names in GpsDeniedSimProperties AND keys in .properties file
    @FXML private TextField latitude;
    @FXML private TextField longitude;
    @FXML private TextField yaw;
    @FXML private TextField altitude;
    @FXML private TextField leaderSpeed;
    @FXML private TextField leaderFlightDistance;
    @FXML private TextField observerSideDistance;
    @FXML private TextField midpointAlong;
    @FXML private TextField midpointRight;
    @FXML private Button    okButton;

    public GpsDeniedDialogController(ResourceBundle resources,
                                     GpsDeniedSimProperties properties,
                                     @SuppressWarnings("unused") javafx.stage.Stage stage) {
        this.resources  = resources;
        this.properties = properties;
    }

    @FXML
    public void initialize() {
        latitude.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        longitude.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        yaw.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        altitude.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        leaderSpeed.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        leaderFlightDistance.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        observerSideDistance.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        midpointAlong.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));
        midpointRight.setTextFormatter(new TextFormatter<>(ArduSimTools.doubleFilter));

        okButton.setOnAction(e -> {
            if (properties.storeParameters(buildProperties(), resources)) {
                Platform.setImplicitExit(false);
                Param.simStatus = Param.SimulatorState.STARTING_UAVS;
                okButton.getScene().getWindow().hide();
            } else {
                ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_LOADING_FXML);
            }
        });
    }

    /** Collects current TextField values into a Properties map via reflection. */
    private Properties buildProperties() {
        Properties p = new Properties();
        for (Field field : this.getClass().getDeclaredFields()) {
            String typeName = field.getAnnotatedType().getType().getTypeName();
            if (!typeName.contains("javafx")) { continue; }
            try {
                Method getter = null;
                if (typeName.contains("TextField")) {
                    getter = field.get(this).getClass().getMethod("getCharacters");
                } else if (typeName.contains("ChoiceBox")) {
                    getter = field.get(this).getClass().getMethod("getValue");
                }
                if (getter != null) {
                    p.setProperty(field.getName(), String.valueOf(getter.invoke(field.get(this))));
                }
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                e.printStackTrace();
            }
        }
        return p;
    }
}
