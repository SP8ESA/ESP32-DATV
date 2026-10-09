"""Scalable, clickable channel diagram; no hardware or network side effects."""
from PyQt5 import QtCore, QtGui, QtWidgets
from qo100_bandplan import CHANNELS, SOURCE_URL, UPLINK_MIN, UPLINK_MAX, BEACON_UPLINK, PREFERRED_UPLINK_MIN


class BandplanWidget(QtWidgets.QWidget):
    frequency_selected = QtCore.pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.frequency = 2370.
        self.baud = 250000
        self.setMinimumHeight(190)
        self.setMaximumHeight(205)
        self.setMouseTracking(True)
        self.setToolTip("QO-100 WB spot frequencies · BATC bandplan v3")

    def set_selection(self, frequency, baud):
        self.frequency, self.baud = frequency, baud
        self.update()

    def x(self, frequency):
        return 100 + (frequency - UPLINK_MIN) / (UPLINK_MAX - UPLINK_MIN) * (self.width() - 145)

    def channel_rect(self, channel):
        span = (1.30, .42, .22)[channel.row]
        return QtCore.QRectF(self.x(channel.uplink - span / 2), 51 + channel.row * 31,
                             self.x(channel.uplink + span / 2) - self.x(channel.uplink - span / 2), 25)

    def channel_at(self, point):
        return next((channel for channel in CHANNELS if self.channel_rect(channel).contains(point)), None)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#f1f5f8"))
        p.setFont(QtGui.QFont("Sans Serif", 9))
        p.setPen(QtGui.QColor("#20374b"))
        p.drawText(10, 17, "QO-100 WB · click a channel to set TX")
        x0, x1 = self.x(UPLINK_MIN), self.x(UPLINK_MAX)
        pref = self.x(PREFERRED_UPLINK_MIN)
        if self.baud <= 333000:
            p.fillRect(QtCore.QRectF(pref, 25, x1 - pref, 120), QtGui.QColor("#e2f1e9"))
        p.setPen(QtGui.QColor("#b9c7d0"))
        for step in range(10):
            frequency = UPLINK_MIN + step
            xpos = self.x(frequency)
            p.drawLine(QtCore.QPointF(xpos, 40), QtCore.QPointF(xpos, 140))
            p.setPen(QtGui.QColor("#526573"))
            p.drawText(QtCore.QRectF(xpos - 31, 23, 62, 20), QtCore.Qt.AlignCenter, f"{frequency:g}")
            p.drawText(QtCore.QRectF(xpos - 35, 144, 70, 20), QtCore.Qt.AlignCenter, f"{frequency + 8089.5:g}")
            p.setPen(QtGui.QColor("#b9c7d0"))
        p.setPen(QtGui.QColor("#526573"))
        p.drawText(9, 37, "TX MHz")
        p.drawText(9, 158, "DL MHz")
        for row, label in enumerate(("1M", "250–500k", "33–125k")):
            p.drawText(QtCore.QRectF(5, 51 + row * 31, 68, 25), QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, label)
        beacon = self.x(BEACON_UPLINK)
        p.setPen(QtGui.QPen(QtGui.QColor("#9b5368"), 1, QtCore.Qt.DashLine))
        p.drawLine(QtCore.QPointF(beacon, 43), QtCore.QPointF(beacon, 142))
        p.drawText(QtCore.QRectF(beacon - 36, 89, 72, 20), QtCore.Qt.AlignCenter, "Beacon")
        if UPLINK_MIN <= self.frequency <= UPLINK_MAX:
            xpos = self.x(self.frequency)
            p.setPen(QtGui.QPen(QtGui.QColor("#18765b"), 2))
            p.drawLine(QtCore.QPointF(xpos, 43), QtCore.QPointF(xpos, 143))
        for channel in CHANNELS:
            selected = abs(channel.uplink - self.frequency) < .000001
            compatible = channel.supports(self.baud)
            fill = "#18765b" if selected else "#d6eadf" if compatible and channel.preferred(self.baud) else "#dbe6f2" if compatible else "#e8edf1"
            p.setPen(QtGui.QPen(QtGui.QColor("#18765b" if selected else "#9baebb"), 1))
            p.setBrush(QtGui.QColor(fill))
            rect = self.channel_rect(channel)
            p.drawRoundedRect(rect, 3, 3)
            p.setPen(QtGui.QColor("white" if selected else "#20374b" if compatible else "#70818d"))
            p.setFont(QtGui.QFont("Sans Serif", 8 if channel.row == 2 else 9))
            p.drawText(rect, QtCore.Qt.AlignCenter, channel.name[1:] if channel.row == 2 else channel.name)
        if UPLINK_MIN <= self.frequency <= UPLINK_MAX:
            label = f"TX {self.frequency:.3f}  →  DL {self.frequency + 8089.5:.3f} MHz"
        else:
            label = f"TX {self.frequency:.3f} MHz · local frequency"
        p.setFont(QtGui.QFont("Sans Serif", 9))
        p.setPen(QtGui.QColor("#20374b"))
        p.drawText(9, 180, label)
        if self.baud <= 333000:
            p.drawText(QtCore.QRectF(self.width() - 225, 167, 215, 20), QtCore.Qt.AlignRight, "Green: preferred narrow section")

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            channel = self.channel_at(event.pos())
            if channel:
                self.frequency_selected.emit(channel.uplink)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        channel = self.channel_at(event.pos())
        if channel:
            self.setCursor(QtCore.Qt.PointingHandCursor)
            rates = "/".join(str(rate // 1000) for rate in channel.rates)
            QtWidgets.QToolTip.showText(event.globalPos(), f"{channel.name} · TX {channel.uplink:.3f} MHz\nDL {channel.downlink:.3f} MHz · {rates} kS/s", self)
        else:
            self.unsetCursor()
            QtWidgets.QToolTip.hideText()
        super().mouseMoveEvent(event)
