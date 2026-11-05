import math
from PyQt5 import QtCore, QtGui, QtWidgets


class MapCanvas(QtWidgets.QGraphicsView):
    poseDrawn=QtCore.pyqtSignal(str,float,float,float)
    operationCanceled=QtCore.pyqtSignal()
    cursorWorld=QtCore.pyqtSignal(float,float)
    def __init__(self):
        super().__init__()
        self.setScene(QtWidgets.QGraphicsScene(self));self.setBackgroundBrush(QtGui.QColor('#101c28'))
        self.setRenderHint(QtGui.QPainter.Antialiasing)
        self.setTransformationAnchor(self.AnchorUnderMouse)
        self.setDragMode(self.ScrollHandDrag);self.setMouseTracking(True)
        self.document=None;self.image_item=None;self.overlays=[];self.preview=None
        self.mode=None;self.anchor=None;self.pan_position=None
        self.setMinimumSize(480,380)

    def set_document(self,doc,fit=True):
        self.document=doc
        if self.image_item:self.scene().removeItem(self.image_item)
        rgb=doc.rgb
        image=QtGui.QImage(rgb.data,doc.width,doc.height,rgb.strides[0],QtGui.QImage.Format_RGB888).copy()
        self.image_item=self.scene().addPixmap(QtGui.QPixmap.fromImage(image));self.image_item.setZValue(-10)
        t=QtGui.QTransform();t.translate(doc.origin[0],-doc.origin[1]);t.rotate(-math.degrees(doc.origin[2]))
        t.scale(doc.resolution,doc.resolution);t.translate(0,-doc.height)
        self.image_item.setTransform(t)
        self.scene().setSceneRect(self.image_item.sceneBoundingRect().adjusted(-.3,-.3,.3,.3))
        if fit:self.fit_map()

    def fit_map(self):
        if self.image_item:self.fitInView(self.image_item.sceneBoundingRect().adjusted(-.3,-.3,.3,.3),QtCore.Qt.KeepAspectRatio)
    def wheelEvent(self,event):
        factor=1.2 if event.angleDelta().y()>0 else 1/1.2
        if 1 < self.transform().m11()*factor < 3000:self.scale(factor,factor)
    def tool(self,mode):
        self.mode=mode;self.anchor=None;self.pan_position=None;self.clear_preview()
        self.setDragMode(self.NoDrag if mode else self.ScrollHandDrag)
        self.viewport().setCursor(QtCore.Qt.CrossCursor if mode else QtCore.Qt.OpenHandCursor)
    def clear_preview(self):
        if self.preview:self.scene().removeItem(self.preview);self.preview=None
    def world(self,event):
        p=self.mapToScene(event.pos());return p.x(),-p.y()
    def mousePressEvent(self,event):
        if event.button()==QtCore.Qt.RightButton:
            self.tool(None);self.operationCanceled.emit();event.accept();return
        if event.button()==QtCore.Qt.MiddleButton:
            self.anchor=None;self.clear_preview();self.pan_position=event.pos()
            self.viewport().setCursor(QtCore.Qt.ClosedHandCursor);event.accept();return
        if self.mode and event.button()==QtCore.Qt.LeftButton:
            x,y=self.world(event)
            if self.document and self.document.contains(x,y):self.anchor=(x,y)
            return
        super().mousePressEvent(event)
    def mouseMoveEvent(self,event):
        if self.pan_position is not None:
            delta=event.pos()-self.pan_position;self.pan_position=event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value()-delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value()-delta.y())
            event.accept();return
        x,y=self.world(event);self.cursorWorld.emit(x,y)
        if self.mode and self.anchor:
            self.clear_preview();a,b=self.anchor
            pen=QtGui.QPen(QtGui.QColor('#fbbf24'),2);pen.setCosmetic(True)
            self.preview=self.scene().addLine(a,-b,x,-y,pen)
        else:super().mouseMoveEvent(event)
    def mouseReleaseEvent(self,event):
        if event.button()==QtCore.Qt.MiddleButton:
            self.pan_position=None
            self.viewport().setCursor(QtCore.Qt.CrossCursor if self.mode else QtCore.Qt.OpenHandCursor)
            event.accept();return
        if self.mode and self.anchor and event.button()==QtCore.Qt.LeftButton:
            x,y=self.world(event);a,b=self.anchor;mode=self.mode
            # Heading tools require a meaningful drag; a click does not silently set yaw=0.
            if mode not in ('socket','new_station') and math.hypot(x-a,y-b)<.03:
                self.anchor=None;self.clear_preview();return
            yaw=0. if mode=='new_station' and math.hypot(x-a,y-b)<.03 else math.atan2(y-b,x-a);self.tool(None)
            self.poseDrawn.emit(mode,a,b,yaw);return
        super().mouseReleaseEvent(event)

    def draw(self,stations,selected,pose=None,scan=(),path=(),target=None,initial=None):
        for item in self.overlays:self.scene().removeItem(item)
        self.overlays=[]
        def line(a,b,color,width=2,dash=False):
            pen=QtGui.QPen(QtGui.QColor(color),width);pen.setCosmetic(True)
            if dash:pen.setStyle(QtCore.Qt.DashLine)
            item=self.scene().addLine(a[0],-a[1],b[0],-b[1],pen);self.overlays.append(item)
        def text(x,y,value,color):
            item=self.scene().addSimpleText(value);font=QtGui.QFont();font.setPointSize(9);item.setFont(font);item.setBrush(QtGui.QColor(color))
            item.setFlag(item.ItemIgnoresTransformations);item.setPos(x,-y);self.overlays.append(item)
        def arrow(p,color,label):
            x,y,angle=p
            tip=(x+.5*math.cos(angle),y+.5*math.sin(angle))
            line((x,y),tip,color,3)
            for side in (-1,1):
                line(tip,(tip[0]-.12*math.cos(angle+side*.5),tip[1]-.12*math.sin(angle+side*.5)),color,3)
            if label:text(x+.06,y-.08,label,color)
        if len(path)>1:
            p=QtGui.QPainterPath(QtCore.QPointF(path[0][0],-path[0][1]))
            for x,y in path[1:]:p.lineTo(x,-y)
            pen=QtGui.QPen(QtGui.QColor('#0ea5e9'),2);pen.setCosmetic(True)
            self.overlays.append(self.scene().addPath(p,pen))
        if scan:
            p=QtGui.QPainterPath()
            for x,y in scan:p.addEllipse(QtCore.QPointF(x,-y),.018,.018)
            self.overlays.append(self.scene().addPath(p,QtGui.QPen(QtCore.Qt.NoPen),QtGui.QBrush(QtGui.QColor('#f97316'))))
        for i,station in enumerate(stations):
            socket,dock=station['socket'],station['pre_dock'];name=('▶ ' if i==selected else '')+station['name']
            if socket:
                x,y=socket['x_m'],socket['y_m']
                line((x-.08,y-.08),(x+.08,y+.08),'#c084fc',3)
                line((x-.08,y+.08),(x+.08,y-.08),'#c084fc',3)
                text(x+.08,y+.45,name+' · 充电口','#c084fc')
            if dock:
                arrow((dock['x_m'],dock['y_m'],dock['yaw_rad']),'#0284c7',name+' · 预停靠')
            if socket and dock:line((socket['x_m'],socket['y_m']),(dock['x_m'],dock['y_m']),'#c084fc',1,True)
        if pose:
            x,y,a=pose;c,s=math.cos(a),math.sin(a)
            corners=[(-.15,-.38),(.67,-.38),(.67,.38),(-.15,.38)]
            poly=QtGui.QPolygonF([QtCore.QPointF(x+c*u-s*v,-(y+s*u+c*v)) for u,v in corners])
            pen=QtGui.QPen(QtGui.QColor('#16a34a'),2);pen.setCosmetic(True)
            self.overlays.append(self.scene().addPolygon(poly,pen,QtGui.QBrush(QtGui.QColor(34,197,94,85))))
            arrow(pose,'#15803d','底盘')
        if target:arrow(target,'#ef4444','导航目标')
        if initial:
            x,y,a=initial;c,s=math.cos(a),math.sin(a)
            corners=[(-.15,-.38),(.67,-.38),(.67,.38),(-.15,.38)]
            points=[(x+c*u-s*v,y+s*u+c*v) for u,v in corners]
            for start,end in zip(points,points[1:]+points[:1]):line(start,end,'#fbbf24',2,True)
            arrow(initial,'#fbbf24','初始位姿 · 配置车体轮廓')
