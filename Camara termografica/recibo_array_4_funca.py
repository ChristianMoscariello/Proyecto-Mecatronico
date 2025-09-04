from flask import Flask, request, jsonify
import numpy as np

app = Flask(__name__)

# Lista global para almacenar los datos de temperatura de un frame completo.
# Esta lista persistirá mientras el servidor esté corriendo.
full_frame_data = []

@app.route("/recibir", methods=["POST"])
def recibir_datos_termicos():
    global full_frame_data
    
    # Obtener el JSON enviado por el M5StickC
    data = request.json
    
    if not data or "temperaturas" not in data or "bloque" not in data:
        return jsonify({"estado": "error", "mensaje": "JSON inválido"}), 400

    # Extraer los datos del JSON
    temperaturas_bloque = data["temperaturas"]
    numero_bloque = data["bloque"]
    
    # Si es el primer bloque (bloque 0), significa que empezamos un nuevo frame.
    # Limpiamos la lista para recibir los nuevos datos.
    if numero_bloque == 0:
        full_frame_data = []
        print("\n--- Recibiendo nuevo frame ---")

    # Añadimos las temperaturas del bloque actual a nuestra lista global
    full_frame_data.extend(temperaturas_bloque)
    
    print(f"Recibido bloque {numero_bloque + 1}/{data['total_bloques']}. Total de lecturas acumuladas: {len(full_frame_data)}")

    # Comprobar si ya hemos recibido el frame completo (768 lecturas)
    if len(full_frame_data) >= 768:
        print("\n¡Frame completo recibido!")
        
        # Convertir la lista a un array de NumPy para facilitar los cálculos
        frame_array = np.array(full_frame_data)
        
        # 1. Calcular el promedio de todo el frame
        promedio = np.mean(frame_array)
        
        # 2. Encontrar la temperatura máxima
        maxima = np.max(frame_array)
        
        # 3. Encontrar la temperatura mínima
        minima = np.min(frame_array)
        
        print(f"  - Análisis del frame:")
        print(f"    - Temperatura Promedio: {promedio:.2f} °C")
        print(f"    - Temperatura Máxima:   {maxima:.2f} °C")
        print(f"    - Temperatura Mínima:   {minima:.2f} °C")
        
        # Opcional: Aquí podrías guardar el array en un archivo,
        # mostrar una imagen térmica, etc.
        
        # Limpiamos la lista para el siguiente frame, aunque el bloque 0 también lo hace.
        full_frame_data = []

    # Responder al M5StickC que todo ha ido bien
    return jsonify({"estado": "OK"}), 200

if __name__ == "__main__":
    # Para instalar las librerías necesarias, ejecuta en tu terminal:
    # pip install Flask numpy
    
    print("Iniciando servidor Flask...")
    print("Escuchando en http://0.0.0.0:6001")
    print("El M5StickC debe apuntar a la IP de esta máquina en la red local.")
    app.run(host="0.0.0.0", port=6001, debug=True)
