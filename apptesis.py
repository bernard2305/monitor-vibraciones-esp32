"""Visor de aceleracion y giro para capturas CSV de cuatro MPU6050."""

from pathlib import Path
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import numpy as np
import pandas as pd
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.widgets import SpanSelector
from scipy.signal import butter, sosfiltfilt


SENSORES = {
    "M1_S1": ("S1 · Esquina superior izquierda", "#1f77b4"),
    "M1_S2": ("S2 · Esquina inferior izquierda", "#ff7f0e"),
    "M2_S3": ("S3 · Esquina inferior derecha", "#2ca02c"),
    "M2_S4": ("S4 · Esquina superior derecha", "#d62728"),
}
EJES = {
    "Aceleración X (lateral)": ("accel_x", "g"),
    "Aceleración Y (frontal)": ("accel_y", "g"),
    "Aceleración Z (vertical)": ("accel_z", "g"),
    "Giroscopio X (cabeceo)": ("gyro_x", "°/s"),
    "Giroscopio Y (alabeo)": ("gyro_y", "°/s"),
    "Giroscopio Z (rotación)": ("gyro_z", "°/s"),
}
ORIGINAL = "Original del CSV"
CENTRADA = "Sin filtro, centrada en reposo"
PASO_BAJO = "Paso bajo 10 Hz"
PASA_BANDA = "Pasa banda 0.5–20 Hz"
COMPLETA = "Captura completa"
EVENTO = "Ventana del evento"
ULTIMOS_10 = "Últimos 10 s"
ULTIMOS_30 = "Últimos 30 s"
FS_NOMINAL = 50.0
SALTO_CSV = 5 / FS_NOMINAL
SENSOR_POR_NOMBRE = {nombre: sid for sid, (nombre, _) in SENSORES.items()}
COLUMNAS_EJES = [columna for columna, _ in EJES.values()]


def leer_captura(ruta):
    df = pd.read_csv(ruta, encoding="utf-8-sig")
    df.columns = df.columns.str.strip().str.lower()
    requeridas = ["timestamp", "sensor_id", *COLUMNAS_EJES]
    faltantes = [columna for columna in requeridas if columna not in df]
    if faltantes:
        raise ValueError("Faltan columnas en el CSV: " + ", ".join(faltantes))

    df = df[requeridas].copy()
    df["sensor_id"] = df["sensor_id"].astype("string").str.strip().str.upper()
    texto = df["timestamp"].astype("string").str.strip()
    fechas = pd.to_datetime(texto, format="%Y-%m-%d %H:%M:%S.%f", errors="coerce")
    if fechas.isna().any():
        sin_fraccion = pd.to_datetime(texto[fechas.isna()], format="%Y-%m-%d %H:%M:%S", errors="coerce")
        fechas = fechas.fillna(sin_fraccion)
    if fechas.notna().any():
        tiempos = fechas
    else:
        solo_hora = texto.where(texto.str.fullmatch(r"\d{2}:\d{2}:\d{2}(\.\d+)?", na=False))
        horas = pd.to_timedelta(solo_hora, errors="coerce").dropna()
        dias = (horas.diff() < -pd.Timedelta(hours=12)).cumsum()
        tiempos = (horas + pd.to_timedelta(dias, unit="D")).reindex(df.index)

    df[COLUMNAS_EJES] = df[COLUMNAS_EJES].apply(pd.to_numeric, errors="coerce")
    df[COLUMNAS_EJES] = df[COLUMNAS_EJES].replace([np.inf, -np.inf], np.nan)
    validas = df["sensor_id"].isin(SENSORES) & tiempos.notna()
    validas &= df[COLUMNAS_EJES].notna().any(axis=1)
    omitidas = int((~validas).sum())
    df = df.loc[validas].copy()
    if df.empty:
        raise ValueError("El CSV no contiene muestras válidas de S1 a S4.")

    tiempos = tiempos[validas]
    df["t_s"] = (tiempos - tiempos.min()).dt.total_seconds()
    return {
        sid: grupo.sort_values("t_s", kind="stable").reset_index(drop=True)
        for sid, grupo in df.groupby("sensor_id", sort=False)
    }, omitidas


def en_intervalo(tiempos, intervalo):
    return (tiempos >= intervalo[0]) & (tiempos <= intervalo[1])


def tramos_validos(tiempos, valores):
    indices = np.flatnonzero(np.isfinite(valores))
    if not indices.size:
        return []
    cortes = (np.diff(indices) > 1) | (np.diff(tiempos[indices]) > SALTO_CSV)
    extremos = np.r_[0, np.flatnonzero(cortes) + 1, len(indices)]
    return [slice(indices[a], indices[b - 1] + 1) for a, b in zip(extremos, extremos[1:])]


def procesar(tiempos, original, reposo, filtro):
    original = np.asarray(original, dtype=float)
    base = original[en_intervalo(tiempos, reposo) & np.isfinite(original)]
    nivel = float(base.mean()) if base.size >= 2 else None
    avisos = []
    if nivel is None:
        avisos.append("reposo sin suficientes muestras")

    if filtro == ORIGINAL:
        return original, nivel, avisos
    if nivel is None:
        return np.full_like(original, np.nan), nivel, avisos

    senal = original - nivel
    if filtro == CENTRADA:
        return senal, nivel, avisos

    if filtro == PASO_BAJO:
        sos = butter(4, 10.0, btype="lowpass", fs=FS_NOMINAL, output="sos")
    else:
        sos = butter(4, [0.5, 20.0], btype="bandpass", fs=FS_NOMINAL, output="sos")
    # Longitud de extensión que usa sosfiltfilt para estas secciones.
    ceros_en_origen = min(np.count_nonzero(sos[:, 2] == 0), np.count_nonzero(sos[:, 5] == 0))
    padlen = 3 * (2 * len(sos) + 1 - ceros_en_origen)
    filtrada = np.full_like(senal, np.nan)
    cortos = 0
    for tramo in tramos_validos(tiempos, senal):
        if tramo.stop - tramo.start <= padlen:
            cortos += 1
        else:
            filtrada[tramo] = sosfiltfilt(sos, senal[tramo], padlen=padlen)
    if cortos:
        avisos.append(f"{cortos} tramo(s) demasiado corto(s) para filtrar")
    return filtrada, nivel, avisos


class AnalizadorSismicoPro:
    def __init__(self, root):
        self.root = root
        root.title("Centro de Análisis Sísmico PRO")
        root.geometry("1200x800")
        root.minsize(850, 650)

        self.captura = {}
        self.duracion = 0.0
        self.reposo = (0.0, 2.0)
        self.evento = None
        self.selector_evento = None

        panel = ttk.Frame(root, padding=(12, 10, 4, 10))
        panel.pack(side=tk.LEFT, fill=tk.Y)
        ttk.Button(panel, text="Cargar CSV de microSD", command=self.cargar_captura).pack(fill=tk.X)
        self.lbl_estado = ttk.Label(panel, text="Esperando archivo…", wraplength=250)
        self.lbl_estado.pack(fill=tk.X, pady=(5, 8))

        self.sensor_var = self.selector(panel, "Sensor", list(SENSOR_POR_NOMBRE), next(iter(SENSOR_POR_NOMBRE)))
        self.eje_var = self.selector(panel, "Eje", list(EJES), "Aceleración Z (vertical)")
        self.tipo_filtro = self.selector(panel, "Filtro", [ORIGINAL, CENTRADA, PASO_BAJO, PASA_BANDA], CENTRADA)
        self.zoom_var = self.selector(panel, "Vista", [COMPLETA, EVENTO, ULTIMOS_10, ULTIMOS_30], COMPLETA)

        reposo, self.reposo_entradas = self.controles_intervalo(panel, "Reposo (s)", self.aplicar_reposo)
        reposo.pack(fill=tk.X, pady=(4, 5))
        evento, self.evento_entradas = self.controles_intervalo(panel, "Evento (s)", self.aplicar_evento)
        evento.pack(fill=tk.X)
        ttk.Button(evento, text="Quitar evento", command=self.quitar_evento).grid(
            row=2, column=0, columnspan=3, sticky=tk.EW, pady=(5, 0)
        )
        ttk.Label(panel, text="También puedes arrastrar sobre Visor de ondas para seleccionar el evento.",
                  wraplength=250).pack(fill=tk.X, pady=(5, 6))

        self.stats_label = ttk.Label(panel, text="Carga una captura para ver las estadísticas.", wraplength=250)
        self.stats_label.pack(fill=tk.X)

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        self.fig_onda, self.canvas_onda = self.crear_grafica("Visor de ondas")
        self.ax_onda = self.fig_onda.subplots()
        self.fig_hist, self.canvas_hist = self.crear_grafica("Distribución")
        self.ax_hist = self.fig_hist.subplots()
        self.fig_m1, self.canvas_m1 = self.crear_grafica("Comparativa M1")
        self.ax_m1 = self.fig_m1.subplots()
        self.fig_m2, self.canvas_m2 = self.crear_grafica("Comparativa M2")
        self.ax_m2 = self.fig_m2.subplots()
        self.fig_4s, self.canvas_4s = self.crear_grafica("Vista 4 sensores")
        self.axs_4s = self.fig_4s.subplots(4, 1, sharex=True, sharey=True)
        for ax in (self.ax_onda, self.ax_hist, self.ax_m1, self.ax_m2, *self.axs_4s):
            self.mensaje_grafica(ax, "Esperando datos…")
            ax.set_axis_off()

    def selector(self, panel, titulo, valores, inicio):
        ttk.Label(panel, text=titulo).pack(anchor=tk.W)
        variable = tk.StringVar(value=inicio)
        combo = ttk.Combobox(panel, textvariable=variable, values=valores, state="readonly", width=32)
        combo.pack(fill=tk.X, pady=(1, 5))
        combo.bind("<<ComboboxSelected>>", lambda _event: self.actualizar_vistas())
        return variable

    def controles_intervalo(self, panel, titulo, aplicar):
        grupo = ttk.LabelFrame(panel, text=titulo, padding=(5, 2, 5, 5))
        entradas = []
        for columna, texto in enumerate(("Inicio", "Fin")):
            ttk.Label(grupo, text=texto).grid(row=0, column=columna, sticky=tk.W)
            variable = tk.StringVar()
            campo = ttk.Entry(grupo, textvariable=variable, width=8)
            campo.grid(row=1, column=columna, sticky=tk.EW, padx=(0, 5))
            campo.bind("<Return>", lambda _event: aplicar())
            grupo.columnconfigure(columna, weight=1)
            entradas.append(variable)
        ttk.Button(grupo, text="Aplicar", command=aplicar).grid(row=1, column=2, sticky=tk.E)
        return grupo, entradas

    def crear_grafica(self, titulo):
        tab = ttk.Frame(self.notebook)
        self.notebook.add(tab, text=titulo)
        figura = Figure(figsize=(8, 6), dpi=100, layout="constrained")
        canvas = FigureCanvasTkAgg(figura, master=tab)
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        return figura, canvas

    def cargar_captura(self):
        ruta = filedialog.askopenfilename(
            parent=self.root, title="Selecciona el CSV de la microSD", filetypes=[("Archivos CSV", "*.csv")]
        )
        if not ruta:
            return
        try:
            captura, omitidas = leer_captura(ruta)
        except (OSError, ValueError) as exc:
            messagebox.showerror("No se pudo cargar el CSV", str(exc), parent=self.root)
            return

        self.captura = captura
        self.duracion = max(float(datos["t_s"].iloc[-1]) for datos in captura.values())
        self.reposo = (0.0, min(2.0, self.duracion))
        self.evento = None
        self.escribir_intervalo(self.reposo_entradas, self.reposo)
        self.escribir_intervalo(self.evento_entradas, None)
        self.zoom_var.set(COMPLETA)
        if SENSOR_POR_NOMBRE[self.sensor_var.get()] not in captura:
            self.sensor_var.set(SENSORES[next(sid for sid in SENSORES if sid in captura)][0])
        estado = f"{Path(ruta).name}\n{len(captura)}/4 sensores · {self.duracion:.2f} s"
        if omitidas:
            estado += f" · {omitidas} fila(s) omitida(s)"
        self.lbl_estado.config(text=estado)
        self.actualizar_vistas()

    @staticmethod
    def escribir_intervalo(entradas, intervalo):
        textos = ("", "") if intervalo is None else [f"{valor:.6f}".rstrip("0").rstrip(".") for valor in intervalo]
        for variable, texto in zip(entradas, textos):
            variable.set(texto)

    def leer_intervalo(self, entradas):
        if not self.captura:
            messagebox.showinfo("Seleccionar intervalo", "Primero carga un CSV.", parent=self.root)
            return None
        try:
            inicio, fin = (float(variable.get().strip().replace(",", ".")) for variable in entradas)
            # Los campos muestran hasta seis decimales.
            if 0 <= inicio < fin <= self.duracion + 0.000001:
                return inicio, min(fin, self.duracion)
        except ValueError:
            pass
        messagebox.showwarning("Intervalo inválido", f"Usa un inicio y un fin entre 0 y {self.duracion:.6f} s.",
                               parent=self.root)
        return None

    def aplicar_reposo(self):
        intervalo = self.leer_intervalo(self.reposo_entradas)
        if intervalo is not None:
            self.reposo = intervalo
            self.actualizar_vistas()

    def aplicar_evento(self):
        intervalo = self.leer_intervalo(self.evento_entradas)
        if intervalo is not None:
            self.seleccionar_evento(*intervalo)

    def seleccionar_evento(self, inicio, fin):
        inicio, fin = max(0.0, inicio), min(fin, self.duracion)
        if inicio < fin:
            self.evento = (inicio, fin)
            self.escribir_intervalo(self.evento_entradas, self.evento)
            self.zoom_var.set(EVENTO)
            self.actualizar_vistas()

    def quitar_evento(self):
        self.evento = None
        self.escribir_intervalo(self.evento_entradas, None)
        if self.zoom_var.get() == EVENTO:
            self.zoom_var.set(COMPLETA)
        self.actualizar_vistas()

    def ventana_visible(self):
        modo = self.zoom_var.get()
        if modo == EVENTO:
            if self.evento is not None:
                return self.evento
            self.zoom_var.set(COMPLETA)
        segundos = {ULTIMOS_10: 10, ULTIMOS_30: 30}.get(modo, self.duracion)
        return max(0.0, self.duracion - segundos), self.duracion

    def actualizar_vistas(self):
        if not self.captura:
            return
        eje = self.eje_var.get()
        columna, unidad = EJES[eje]
        filtro = self.tipo_filtro.get()
        ventana = self.ventana_visible()
        series = {}
        for sid, df in self.captura.items():
            tiempos = df["t_s"].to_numpy()
            valores, nivel, avisos = procesar(tiempos, df[columna].to_numpy(), self.reposo, filtro)
            en_reposo = valores[en_intervalo(tiempos, self.reposo) & np.isfinite(valores)]
            rms = float(en_reposo.std()) if en_reposo.size >= 2 else None
            pico = None
            if self.evento is not None and nivel is not None:
                indices = np.flatnonzero(en_intervalo(tiempos, self.evento) & np.isfinite(valores))
                if indices.size:
                    referencia = nivel if filtro == ORIGINAL else 0.0
                    amplitudes = np.abs(valores[indices] - referencia)
                    i = int(np.argmax(amplitudes))
                    indice = indices[i]
                    pico = (tiempos[indice], valores[indice], amplitudes[i])
            series[sid] = (tiempos, valores, rms, pico, avisos)

        sid = SENSOR_POR_NOMBRE[self.sensor_var.get()]
        self.dibujar_individual(sid, series, ventana, eje, unidad, filtro)
        self.dibujar_comparativa(self.ax_m1, self.canvas_m1, ("M1_S1", "M1_S2"), series, ventana, eje, unidad, filtro)
        self.dibujar_comparativa(self.ax_m2, self.canvas_m2, ("M2_S3", "M2_S4"), series, ventana, eje, unidad, filtro)
        self.dibujar_cuatro(series, ventana, eje, unidad, filtro)

    @staticmethod
    def mensaje_grafica(ax, mensaje):
        ax.text(0.5, 0.5, mensaje, ha="center", va="center", transform=ax.transAxes, color="#616971")

    def preparar_onda(self, ax, ventana, unidad, filtro):
        ax.clear()
        ax.set_axis_on()
        inicio, fin = ventana
        if inicio == fin:
            inicio, fin = inicio - 0.5, fin + 0.5
        ax.set_xlim(inicio, fin)
        if filtro != ORIGINAL:
            ax.axhline(0, linewidth=0.7, color="#6a737b", linestyle="--")
        if self.evento is not None and ventana != self.evento:
            ax.axvspan(*self.evento, color="#cfe4f4", alpha=0.5, zorder=0)
        ax.set_ylabel(unidad)
        ax.grid(alpha=0.25)

    def dibujar_senal(self, ax, sid, serie, ventana, marcar_pico=False):
        if serie is None:
            return False
        tiempos, valores, _, pico, _ = serie
        visibles = en_intervalo(tiempos, ventana)
        tiempos, valores = tiempos[visibles], valores[visibles]
        tramos = tramos_validos(tiempos, valores)
        if not tramos:
            return False
        nombre, color = SENSORES[sid]
        cortes = [tramo.start for tramo in tramos[1:]]
        ax.plot(np.insert(tiempos, cortes, np.nan), np.insert(valores, cortes, np.nan),
                color=color, linewidth=1.1, label=nombre)
        aislados = [tramo.start for tramo in tramos if tramo.stop - tramo.start == 1]
        if aislados:
            ax.plot(tiempos[aislados], valores[aislados], ".", color=color, markersize=4)
        if marcar_pico and pico is not None and ventana[0] <= pico[0] <= ventana[1]:
            ax.scatter(pico[0], pico[1], color="#bd3429", s=22, zorder=4)
        return True

    def dibujar_individual(self, sid, series, ventana, eje, unidad, filtro):
        if self.selector_evento is not None:
            self.selector_evento.set_active(False)
            self.selector_evento.disconnect_events()
        serie = series.get(sid)
        self.preparar_onda(self.ax_onda, ventana, unidad, filtro)
        if not self.dibujar_senal(self.ax_onda, sid, serie, ventana, marcar_pico=True):
            mensaje = "Sin datos en esta vista" if serie else "Sensor ausente en esta captura"
            self.mensaje_grafica(self.ax_onda, mensaje)
        nombre, color = SENSORES[sid]
        self.ax_onda.set_title(f"{nombre}\n{eje} · {filtro}")
        self.ax_onda.set_xlabel("Tiempo de la captura (s)")
        self.selector_evento = SpanSelector(
            self.ax_onda, self.seleccionar_evento, "horizontal", useblit=True, props={"facecolor": "#387ab6", "alpha": 0.3},
            button=1, minspan=0.02, ignore_event_outside=True,
        )

        self.ax_hist.clear()
        self.ax_hist.set_axis_on()
        self.ax_hist.set_title(f"Distribución · {nombre}\n{eje} · {filtro}")
        self.ax_hist.set_xlabel(unidad)
        self.ax_hist.set_ylabel("Número de muestras")
        self.stats_label.config(text="Sin datos del sensor en esta captura.")
        if serie is not None:
            tiempos, valores, rms, pico, avisos = serie
            muestras = valores[en_intervalo(tiempos, ventana) & np.isfinite(valores)]
            if muestras.size:
                self.ax_hist.hist(muestras, bins=35, color=color, alpha=0.75)
            pico_texto = f"{pico[2]:.5f} {unidad} a {pico[0]:.3f} s" if pico else "—"
            rms_texto = f"{rms:.5f} {unidad}" if rms is not None else "—"
            texto = f"RMS en reposo  {rms_texto}\nPico del evento  {pico_texto}\nMuestras en vista  {muestras.size}"
            if filtro in (PASO_BAJO, PASA_BANDA):
                texto += f"\nFiltrado a {FS_NOMINAL:g} Hz nominales"
            if filtro == ORIGINAL:
                texto += "\nPico medido respecto al reposo."
            if self.evento and max(self.evento[0], self.reposo[0]) < min(self.evento[1], self.reposo[1]):
                texto += "\nReposo y evento se superponen."
            if avisos:
                texto += "\n" + "\n".join(avisos).capitalize()
            self.stats_label.config(text=texto)
        if not self.ax_hist.has_data():
            self.mensaje_grafica(self.ax_hist, "Sin datos en esta vista")
        self.canvas_onda.draw_idle()
        self.canvas_hist.draw_idle()

    def dibujar_comparativa(self, ax, canvas, sensores, series, ventana, eje, unidad, filtro):
        self.preparar_onda(ax, ventana, unidad, filtro)
        faltan = [SENSORES[sid][0].split(" · ")[0] for sid in sensores
                  if not self.dibujar_senal(ax, sid, series.get(sid), ventana)]
        if len(faltan) < len(sensores):
            ax.legend(loc="upper right")
        if faltan:
            ax.text(0.02, 0.98, f"Sin datos en vista: {', '.join(faltan)}", va="top", transform=ax.transAxes)
        ax.set_title(f"{eje} · {filtro}")
        ax.set_xlabel("Tiempo de la captura (s)")
        canvas.draw_idle()

    def dibujar_cuatro(self, series, ventana, eje, unidad, filtro):
        minimo, maximo = np.inf, -np.inf
        for ax, (sid, (nombre, _)) in zip(self.axs_4s, SENSORES.items()):
            serie = series.get(sid)
            self.preparar_onda(ax, ventana, unidad, filtro)
            indicadores = []
            if self.dibujar_senal(ax, sid, serie, ventana, marcar_pico=True):
                tiempos, valores, rms, pico, avisos = serie
                visibles = valores[en_intervalo(tiempos, ventana) & np.isfinite(valores)]
                minimo = min(minimo, visibles.min())
                maximo = max(maximo, visibles.max())
                if rms is not None:
                    indicadores.append(f"RMS reposo {rms:.4g} {unidad}")
                if pico is not None:
                    indicadores.append(f"Pico evento {pico[2]:.4g} {unidad}")
                if avisos and filtro in (PASO_BAJO, PASA_BANDA):
                    indicadores.append("Filtrado parcial")
            else:
                mensaje = "Sin datos en esta vista" if serie else "Sensor ausente en esta captura"
                if serie and serie[4]:
                    mensaje += "\n" + ". ".join(serie[4]).capitalize()
                self.mensaje_grafica(ax, mensaje)
            titulo = nombre + "\n" + "   ·   ".join(indicadores) if indicadores else nombre
            ax.set_title(titulo, loc="left", fontsize=9)

        if np.isfinite(minimo):
            if filtro == ORIGINAL:
                margen = max(0.07 * (maximo - minimo), 0.00001)
                self.axs_4s[-1].set_ylim(minimo - margen, maximo + margen)
            else:
                limite = max(1.08 * max(abs(minimo), abs(maximo)), 0.00001)
                self.axs_4s[-1].set_ylim(-limite, limite)
        else:
            self.axs_4s[-1].set_ylim(-1, 1)
        self.fig_4s.suptitle(f"{eje} · {filtro}", fontsize=12)
        self.axs_4s[-1].set_xlabel("Tiempo de la captura (s)")
        self.canvas_4s.draw_idle()


if __name__ == "__main__":
    try:
        from ctypes import windll

        windll.shcore.SetProcessDpiAwareness(1)
    except (ImportError, AttributeError, OSError):
        pass
    root = tk.Tk()
    app = AnalizadorSismicoPro(root)
    root.mainloop()
