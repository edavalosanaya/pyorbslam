import logging

from aiohttp import web

from ..async_loop_thread import AsyncLoopThread
from .comm_bus import CommBus

logger = logging.getLogger('pyorbslam')

class HttpServer:

    def __init__(self, port: int, cbus: CommBus):

        # Saving input parameters
        self.port = port
        self.cbus = cbus

        # Creating app
        self.app = web.Application()
 
        # Create the async loop thread
        self._thread = AsyncLoopThread()
        self._thread.start()

        # Adding routes
        self.app.add_routes([
            web.get("/", self.hello),
            web.get("/shutdown", self.shutdown_server),
            web.post("/config/zeromq", self.config_zeromq),
            web.post("/visuals/create", self.create_visual),
            web.post("/visuals/delete", self.delete_visual)
        ])

        # Run in an AsyncLoopThread
        self._thread.exec(self.start()).result(timeout=10)
    
    def __str__(self):
        return "<HttpServer>"

    def __repr__(self):
        return str(self)
        
    async def start(self):

        # Start the TCP site
        self._runner = web.AppRunner(self.app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, "0.0.0.0", self.port)
        await self._site.start()

        # Get the new port
        self.port = self._site._server.sockets[0].getsockname()[1]
        logger.debug(f"{self}: Running at localhost:{self.port}")

    async def hello(self, request):
        return web.Response(text="Hello World!")

    async def shutdown_server(self, request):
        logger.debug(f"{self}: shutting down!")
        self.cbus.closeApp.emit()
        return web.HTTPOk()

    async def config_zeromq(self, request):
        # Obtain information, json={'ip': self._client_host, 'port': self._zmq_port}
        data = await request.json()
        self.cbus.zeromqSub.emit(str(data['ip']), int(data['port']))
        return web.HTTPOk()

    async def create_visual(self, request):
        # Obtain information, json={'name': name, 'vtype': vtype}
        data = await request.json()
        self.cbus.visualCreate.emit(data['name'], data['vtype'])
        return web.HTTPOk()

    async def delete_visual(self, request):
        # Obtain information, json={'name': name}
        data = await request.json()
        self.cbus.visualDelete.emit(data['name'])
        return web.HTTPOk()

    def stop(self):
        self._thread.stop()
