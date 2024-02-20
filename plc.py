import snap7
import struct

# Configurar la conexión con el PLC
plc_ip = '192.168.1.12'
rack = 0
slot = 1

# Instancia de cliente
cliente = snap7.client.Client()

# Conexión a PLC
cliente.connect(plc_ip, rack, slot)

if cliente.get_connected():
    print("Conexión exitosa al PLC")

    try:
        # Solicitar al usuario el número del DB y la dirección
        db_number = 1
        start = 4

        # Leer desde el PLC
        data = cliente.db_read(db_number, start, 4)  # Leemos 8 bytes, que es el tamaño de un double

        # Convertir los datos al tipo de datos deseado (en este ejemplo a float)
        valor_leido = struct.unpack('>I', data)[0]

        print(f"Valor leído desde DB{db_number}.DBD{start}: {valor_leido}")

    except Exception as e:
        print(f"Error: {e}")

    # Desconectar del PLC
    cliente.disconnect()
else:
    print("Error al conectar al PLC")
