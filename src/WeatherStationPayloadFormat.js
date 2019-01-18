function Decoder(bytes, port) {
    var decoded = {};
    
    decoded.moisture = bytes[0];
    decoded.light = bytes[1];
    decoded.temperature = ((bytes[2] << 8) | bytes[3]) / 100.00;
    decoded.pressure = 900 + ((bytes[4] << 8) | bytes[5]) / 100.00;
    decoded.humidity = ((bytes[6] << 8) | bytes[7]) / 100.00;
    decoded.gas = ((bytes[8] << 8) | bytes[9]) / 100.00;
    decoded.trigger = decoded.temperature > 25.00;
    
    return decoded;
  }