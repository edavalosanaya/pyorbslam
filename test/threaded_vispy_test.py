import multiprocessing as mp
import time

import pyorbslam
from vispy import app, gloo
from vispy.gloo import Program

app.use_app(backend_name='glfw')


def vispy_process():
    vertex = """
        uniform float theta;
        attribute vec4 color;
        attribute vec2 position;
        varying vec4 v_color;
        void main()
        {
            float ct = cos(theta);
            float st = sin(theta);
            float x = 0.75* (position.x*ct - position.y*st);
            float y = 0.75* (position.x*st + position.y*ct);
            gl_Position = vec4(x, y, 0.0, 1.0);
            v_color = color;
        } """

    fragment = """
        varying vec4 v_color;
        void main()
        {
            gl_FragColor = v_color;
        } """

    class Canvas(app.Canvas):
        def __init__(self):
            super().__init__(size=(512, 512), title='Rotating quad',
                             keys='interactive')
            # Build program & data
            self.program = Program(vertex, fragment, count=4)
            self.program['color'] = [(1, 0, 0, 1), (0, 1, 0, 1),
                                     (0, 0, 1, 1), (1, 1, 0, 1)]
            self.program['position'] = [(-1, -1), (-1, +1),
                                        (+1, -1), (+1, +1)]
            self.program['theta'] = 0.0

            gloo.set_viewport(0, 0, *self.physical_size)
            gloo.set_clear_color('white')

            self.timer = app.Timer('auto', self.on_timer)
            self.clock = 0
            self.timer.start()

            self.show()

        def on_draw(self, event):
            gloo.clear()
            self.program.draw('triangle_strip')

        def on_resize(self, event):
            gloo.set_viewport(0, 0, *event.physical_size)

        def on_timer(self, event):
            self.clock += 0.001 * 1000.0 / 60.
            self.program['theta'] = self.clock
            self.update()

    c = Canvas()
    app.run()


# def update_points(queue):
#     while True:
#         # Simulate generating new points
#         new_point = time.time()

#         # Put the new point into the queue
#         queue.put(new_point)

#         # Sleep for a while
#         time.sleep(0.5)

class A():

    def __init__(self):
        # Start the VisPy application process
        self.vispy_proc = mp.Process(target=vispy_process)
        self.vispy_proc.start()

if __name__ == "__main__":
    # Create a multiprocessing Queue for communication between processes
    point_queue = mp.Queue()

    a = A()

    # Start the update process
    # update_proc = mp.Process(target=update_points, args=(point_queue,))
    # update_proc.start()

    # # Run the main process to retrieve points from the queue and update the VisPy application
    # while True:
    #     try:
    #         # Get new point from the queue
    #         new_point = point_queue.get(block=True, timeout=1)

    #         # Print the new point
    #         print(f"Received new point: {new_point}")

    #     except queue.Empty:
    #         # If the queue is empty, exit the loop
    #         break

    # Wait for the processes to finish
    # update_proc.join()
    # time.sleep(10)
    a.vispy_proc.join()
