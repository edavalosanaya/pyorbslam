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
            web.post("/visuals/add", self.add_visual),
            web.post("/visuals/remove", self.remove_visual)
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
        
        # Obtain information
        data = await request.json()
        return web.HTTPOK()

    async def add_visual(self, request):
        ...

    async def remove_visual(self, request):
        ...

    def stop(self):
        self._thread.stop()
