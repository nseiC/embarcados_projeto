// =====================================================================
// No "Function" do Node-RED
// Configurar: Outputs = 2  (saida 1 -> Gauge, saida 2 -> Chart)
// Entrada: msg.payload vindo do MQTT IN (topico casa/sala/temperatura)
//          contendo JSON: {"sensor":"ESP32-01","temperatura":25.4,
//                          "umidade":61,"timestamp":1747580000}
// =====================================================================

let dados;

try {
    dados = typeof msg.payload === "string"
        ? JSON.parse(msg.payload)
        : msg.payload;
} catch (e) {
    node.warn("JSON invalido recebido");
    return null;
}

let temperatura = Number(dados.temperatura);

if (isNaN(temperatura)) {
    node.warn("Campo temperatura invalido ou ausente");
    return null;
}

let msgGauge = {
    payload: temperatura,
    topic: "Temperatura"
};

let msgChart = {
    payload: temperatura,
    topic: "Temperatura"
};

return [msgGauge, msgChart];
