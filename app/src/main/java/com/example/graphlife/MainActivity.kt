package com.example.graphlife

//import com.google.androidgamesdk.GameActivity

// Mapbox

// for plaid token exchange

// https://github.com/android/snippets/blob/5ae1f7852164d98d055b3cc6b463705989cff231/compose/snippets/src/main/java/com/example/compose/snippets/text/TextSnippets.kt#L104-L107
import android.Manifest
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.res.AssetManager
import android.graphics.BlurMaskFilter
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.graphics.PointF
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.IBinder
import android.util.AttributeSet
import android.util.Log
import android.view.MotionEvent
import android.view.View
import android.widget.LinearLayout
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.absolutePadding
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.wrapContentSize
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.DrawModifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.geometry.Rect
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.graphics.ColorMatrix
import androidx.compose.ui.graphics.Shape
import androidx.compose.ui.graphics.drawscope.ContentDrawScope
import androidx.compose.ui.graphics.drawscope.drawIntoCanvas
import androidx.compose.ui.layout.onGloballyPositioned
import androidx.compose.ui.platform.ComposeView
import androidx.compose.ui.res.colorResource
import androidx.compose.ui.text.ExperimentalTextApi
import androidx.compose.ui.text.font.Font
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontVariation
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.app.NotificationCompat
import androidx.work.Worker
import androidx.work.WorkerParameters
import com.example.graphlife.databinding.ActivityMainBinding
import com.google.android.gms.location.FusedLocationProviderClient
import com.google.android.gms.location.LocationServices
import com.mapbox.geojson.Point
import com.mapbox.maps.CameraOptions
import com.mapbox.maps.MapView
import com.mapbox.maps.Style
import com.plaid.link.OpenPlaidLink
import com.plaid.link.configuration.LinkTokenConfiguration
import com.plaid.link.result.LinkExit
import com.plaid.link.result.LinkSuccess
import fuel.Fuel
import fuel.get
import kotlinx.coroutines.*
import kotlinx.coroutines.flow.*

class GraphNode {
    constructor(x: Float, y :Float, radius : Float) {
        this.x = x
        this.y = y
        this.radius = radius
    }
    var x : Float
    var y : Float
    var radius : Float
}

class UIState
{
    constructor(activatedMenuItems : HashSet<String>, menuVisible : Boolean, mapVisible : Boolean, plaidTrigger : Boolean)
    {
        this.activatedMenuItems = activatedMenuItems
//        Log.d("GraphDebug", "ON CONSTRUCT: Activated categories count is ${this.activatedMenuItems.size}")
        this.menuVisible = menuVisible
        this.mapVisible = mapVisible
        this.plaidTrigger = plaidTrigger
    }
    var activatedMenuItems : HashSet<String>
    var menuVisible : Boolean
    var mapVisible : Boolean
    var plaidTrigger : Boolean
}

class Position {
    constructor(x: Float, y :Float) {
        this.x = x
        this.y = y
    }
    var x : Float
    var y : Float
}

// TODO: Render directly in NDK with Skia to avoid this...
class Edge {
    constructor(headX: Float, headY :Float, tailX: Float, tailY :Float, edgeType :Int, val0:Float, val1:Float, val2:Float, val3:Float) {
        this.headX = headX
        this.headY = headY
        this.tailX = tailX
        this.tailY = tailY

        this.edgeType = edgeType

        this.val0 = val0
        this.val1 = val1
        this.val2 = val2
        this.val3 = val3
    }
    var headX : Float
    var headY : Float
    var tailX : Float
    var tailY : Float

    var edgeType : Int

    var val0 : Float
    var val1 : Float
    var val2 : Float
    var val3 : Float
}

class GraphView : View {

    public var parentActivity : AppCompatActivity? = null

    constructor(context: Context) : super(context) {
    }

    constructor(context: Context, attrs: AttributeSet?) : super(context, attrs) {
    }

    fun conicToCubicBezier(p0: PointF, p1: PointF, p2: PointF): List<PointF> {
        val c1 = PointF(p0.x + 2.0f / 3.0f * (p1.x - p0.x), p0.y + 2.0f / 3.0f * (p1.y - p0.y))
        val c2 = PointF(p2.x + 2.0f / 3.0f * (p1.x - p2.x), p2.y + 2.0f / 3.0f * (p1.y - p2.y))

        return listOf(p0, c1, c2, p2)
    }

    // TODO: Refactor to flecs code
    var joystickActive : Boolean = false
    var startJoystickX = 0.0f
    var startJoystickY = 0.0f
    var currentJoystickX = 0.0f
    var currentJoystickY = 0.0f
    var joystickPointerId : Int = 0
//    var playerX : Float = 0.0f;
//    var playerY : Float = 0.0f;
//    var playerMoveSpeed = 0.001f;

    override fun onTouchEvent(event: MotionEvent): Boolean {
//        Log.d("GraphTouchDebug", "GraphAction is ${event?.action}")
        if (event?.action == MotionEvent.ACTION_DOWN)
        {
            joystickActive = true
            joystickPointerId = event.getPointerId(0)
            startJoystickX = event.x
            startJoystickY = event.y
            currentJoystickX = event.x
            currentJoystickY = event.y
        }
        else if (event?.action == MotionEvent.ACTION_MOVE)
        {
            currentJoystickX = event.x
            currentJoystickY = event.y
        }
        else if (event?.action == MotionEvent.ACTION_UP)
        {
            joystickActive = false;
//            for (i in 1..event.pointerCount)
//            {
//                if (event.getPointerId(i) == joystickPointerId)
//                {
//
//                }
//            }
        }
        else if (joystickActive)
        {
            val pointerIndex = event.findPointerIndex(joystickPointerId)
            var coords = MotionEvent.PointerCoords()
            event.getPointerCoords(pointerIndex, coords)
            currentJoystickX = coords.x
            currentJoystickY = coords.y
        }
        return event.action != MotionEvent.ACTION_UP
    }


    var nodes : ArrayList<GraphNode> = ArrayList<GraphNode>()
    var edges : ArrayList<Edge> = ArrayList<Edge>()

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        Log.d("GraphFix", "Draw canvas")

        canvas.save()
        val ctx = context as MainActivity
        val menuBackground = ctx.findViewById<LinearLayout>(R.id.menuBackground)

//        val composeView = menuBackground.findViewById<ComposeView>(R.id.composeView)
//        val firstElementY = composeView.children.firstOrNull()?.layoutCoordinates?.positionInParent()?.y ?: 0f

        // TODO: Okay here's where it gets tricky...
        // We need to Python generate the plecs from the Kotlin UI element static analysis
        // https://developer.android.com/reference/kotlin/androidx/compose/ui/layout/OnGloballyPositionedModifier

        //- 78.0f, - 128.0f
        ctx.setUIMenuOffset(menuBackground.width/2.0f, 0.0f)
        val cameraPos = ctx.getCameraPos()
        Log.d("GraphFix", "Camera pos at  ${cameraPos.x}, ${cameraPos.y}")
        // menuBackground.width/2.0f
        canvas.translate((width/2.0f) - cameraPos.x, (height/2.0f) - cameraPos.y)
//        canvas.translate((width/2.0f), (height/2.0f))
        val spaceCadetGreyColor = context.getColor(R.color.space_cadet_grey)

        val debugPaint = Paint().apply {
            color = Color.BLACK // You can choose any color you like
            style = Paint.Style.STROKE // To draw just the outline of the path
            strokeWidth = 5f // Adjust the line thickness as needed
        }

        val cubicPaint = Paint().apply {
            color = Color.BLUE // You can choose any color you like
            style = Paint.Style.FILL // To draw just the outline of the path
            strokeWidth = 4f // Adjust the line thickness as needed
        }

        val conicPaint = Paint().apply {
            color = Color.RED // You can choose any color you like
            style = Paint.Style.FILL // To draw just the outline of the path
            strokeWidth = 4f // Adjust the line thickness as needed
        }

        // TODO: Create edges here!?
        // TODO: JNI node/edge visual style
        for (edge in edges) {
            val path = Path()
            path.moveTo(edge.headX, edge.headY)
//            Log.e("EdgeDebug", "${edge.edgeType}")
            if (edge.edgeType == 0)
            {
                debugPaint.setColor(Color.BLACK)
                path.lineTo(edge.tailX, edge.tailY)
            } else if (edge.edgeType == 1)
            {
                // path.conicTo(edge.tailX, edge.tailY, edge.val0, edge.val1, edge.val2);
//                canvas.drawCircle(edge.val0, edge.val1, 5.0f, conicPaint)
//                canvas.drawLine(edge.val0, edge.val1, edge.headX, edge.headY, conicPaint)
//                canvas.drawLine(edge.val0, edge.val1, edge.tailX, edge.tailY, conicPaint)
                val points = conicToCubicBezier(PointF(edge.headX, edge.headY), PointF(edge.val0, edge.val1), PointF(edge.tailX, edge.tailY))
                path.cubicTo(points[1].x, points[1].y, points[2].x, points[2].y, edge.tailX, edge.tailY);
//                canvas.drawCircle(points[1].x, points[1].y, 5.0f, cubicPaint)
//                canvas.drawCircle(points[2].x, points[2].y, 5.0f, cubicPaint)
                //path.lineTo(edge.tailX, edge.tailY)
            } else if (edge.edgeType == 2)
            {
                debugPaint.setColor(Color.parseColor("#BBBBBBBB"))
                path.cubicTo(edge.val0, edge.val1, edge.val2, edge.val3, edge.tailX, edge.tailY);
//                canvas.drawCircle(edge.val0, edge.val1, 5.0f, cubicPaint)
//                canvas.drawCircle(edge.val2, edge.val3, 5.0f, cubicPaint)
            }
            canvas.drawPath(path, debugPaint)
        }

//        var radius = 16f
//        var radius = 6f

        val paint = Paint()
        paint.style = Paint.Style.FILL

        paint.color = Color.WHITE
        for (node in nodes) {
            Log.d("GraphFix", "Render node at  ${node.x}, ${node.y}")
            canvas.drawCircle(node.x, node.y, node.radius, paint)
        }

        paint.color = Color.BLACK

        paint.style = Paint.Style.STROKE
        paint.strokeWidth = 5.0f;
        paint.style = Paint.Style.FILL
        for (node in nodes) {
            canvas.drawCircle(node.x, node.y, node.radius, paint)
        }

        paint.style = Paint.Style.FILL
        var radius = 24.0f
        // TODO: Draw player node...
        canvas.restore()
        paint.color = Color.parseColor("#44000000")
        paint.maskFilter = BlurMaskFilter(
            6.0f,
            BlurMaskFilter.Blur.NORMAL
        )

        val playerPos = ctx.getPlayerPos()
        var player_x_pos = width/2.0f - (cameraPos.x - playerPos.x)
        var player_y_pos = height/2.0f - (cameraPos.y - playerPos.y)

//        var player_x_pos = width/2.0f
//        var player_y_pos = height/2.0f

//        var player_y_pos = height/3.0f
        // TODO: Offset camera center when in menu
//        var player_x_pos = width/3.0f-40
//        var player_y_pos = height/3.0f-80
        canvas.drawCircle(player_x_pos, player_y_pos, radius, paint);
        paint.maskFilter = null
        paint.color = Color.WHITE;
        radius = 20.0f
        canvas.drawCircle(player_x_pos, player_y_pos, radius, paint);
        paint.color = Color.RED;
        radius = 16.0f
        canvas.drawCircle(player_x_pos, player_y_pos, radius, paint);

//        radius = 10.0f;
//        paint.style = Paint.Style.FILL
//
//        for (node in nodes) {
//            canvas.drawCircle(node.x, node.y, radius, paint)
//        }

    }

}

// ...

class TrackLocationWorker(context: Context, workerParams: WorkerParameters) : Worker(context,
    workerParams
) {
    override fun doWork(): Result {
        TODO("Not yet implemented")
    }
}

// Foreground service that stores location transition nodes!
class TrackLocationService() : Service()
{
    private val handler = Handler()
    private var isRunning = false
    var lastLongitude : Double = 0.0
    var lastLatitude : Double = 0.0

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        Log.d("GraphLifeServices", "Foreground service started")
        val CHANNEL_ID = "PersonalData"
        val name = getString(R.string.channel_name)
        val descriptionText = getString(R.string.channel_description)
        val importance = NotificationManager.IMPORTANCE_LOW
        val mChannel = NotificationChannel(CHANNEL_ID, name, importance)
        mChannel.description = descriptionText
        val notificationManager = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        notificationManager.createNotificationChannel(mChannel)

        val notification = NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle(getText(R.string.notification_title))
            .setContentText(getText(R.string.notification_msg))
//            .setCategory(Notification.CATEGORY_NAVIGATION)
            .setSmallIcon(R.mipmap.ic_launcher)
            .setChannelId(CHANNEL_ID)
            .setOngoing(false)
            .build()
        startForeground(1, notification)
        // The notification is functional:
        // Now update the text every n seconds...


//        with(NotificationManagerCompat.from(this)) {
//            // notificationId is a unique int for each notification that you must define
//            if (ActivityCompat.checkSelfPermission(
//                    applicationContext,
//                    Manifest.permission.POST_NOTIFICATIONS
//                ) != PackageManager.PERMISSION_GRANTED
//            ) {
//            } else
//            {
//                notify(1, notification)
//            }
//        }
        isRunning = true
        handler.post(trackLocation)
        return super.onStartCommand(intent, flags, startId)
//        return START_STICKY
    }

    // TOOD: Consider using https://kotlinlang.org/docs/channels.html#ticker-channels
    private val trackLocation = object : Runnable {
        override fun run() {
            getLocation()

            if (isRunning) {
                handler.postDelayed(this, 5000) // Get location every second
            }
        }
    }

    override fun onBind(p0: Intent?): IBinder? {
        return null
    }

    override fun onCreate() {
        super.onCreate()
    }

    override fun onDestroy() {
        isRunning = false
        super.onDestroy()
    }

    fun getLocation() {
        val client: FusedLocationProviderClient =
            LocationServices.getFusedLocationProviderClient(applicationContext)

        client.lastLocation
            .addOnSuccessListener { location ->
                if (location != null) {
                    // Print the location via debug
                    Log.d("Location", "Latitude: ${location.latitude}, Longitude: ${location.longitude}")
                    if (location.longitude != lastLongitude && location.latitude != lastLatitude) {
                        // TODO: Flow to GraphLife server
                        // latitude = location.latitude
                        // longitude = location.longitude
                    }
                    lastLongitude = location.longitude
                    lastLatitude = location.latitude
                    val updateMapIntent = Intent("UpdateLocationMap")
//                    updateMapIntent.setData()
                    sendBroadcast(updateMapIntent)
                    // TODO: Send flecs query for locations to GraphLife server
//                    Log.d("Location", "Total location nodes stored: ${locations?.size}")
                } else {
                    // Handle the case where the location is null
                    Log.e("Location", "Location is null")
                }
            }
            .addOnFailureListener { e ->
                // Handle any errors that occurred while getting the location
                Log.e("Location", "Error getting location: $e")
            }

    }
}

var mapView: MapView? = null

@Composable
fun Greeting(name: String) {
    Text(text = "Hello $name!")
}

class MainActivity : AppCompatActivity() {

    private class LocationMapUpdateReceiver : BroadcastReceiver() {

        public var parentActivity : AppCompatActivity? = null

        override fun onReceive(context: Context?, intent: Intent) {
            if (intent.getAction().equals("UpdateLocationMap"))
            {
                Log.d("Location", "Update location from activity!")
                // TODO: Get current location from GraphLife server

//                val camera : CameraOptions = CameraOptions.Builder()
//                    .center(Point.fromLngLat(location.longitude, location.latitude))
//                    .build()

                // Don't dox yourself lmao
//                val camera : CameraOptions = CameraOptions.Builder()
//                    .center(Point.fromLngLat(-122.1701, 37.4277))
//                    .build()

//                mapView = parentActivity?.findViewById<MapView>(R.id.mapView)
//                mapView?.getMapboxMap()?.setCamera(camera)
            }
        }
    }

    private val locationMapUpdateReceiver = LocationMapUpdateReceiver();

    var uiState : UIState = UIState(HashSet<String>(), false, false, false)
    private val handler = Handler()
    private var isRunning = false

    override fun onResume() {
        super.onResume()
        startGameLoop()
        if (locationMapUpdateReceiver != null)
        {
            val intentFilter = IntentFilter("UpdateLocationMap")
            registerReceiver(locationMapUpdateReceiver, intentFilter)
        }
    }

    override fun onPause() {
        super.onPause()
        stopGameLoop()
        if (locationMapUpdateReceiver != null)
        {
            unregisterReceiver(locationMapUpdateReceiver)
        }
    }

    // TODO: Run this request in Spacetime 🌎 menu! :)
    val locationPermissionRequest = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        when {
            permissions.getOrDefault(Manifest.permission.ACCESS_FINE_LOCATION, false) -> {
                // Precise location access granted.
            }
            permissions.getOrDefault(Manifest.permission.ACCESS_COARSE_LOCATION, false) -> {
                // Only approximate location access granted.
            } else -> {
            // No location access granted.
        }
        }
    }

    private lateinit var binding: ActivityMainBinding

    private val gameLoop = object : Runnable {
        override fun run() {
            // Your game logic here
            val graphView = findViewById<GraphView>(R.id.graphView)
            stepWorld()
            uiState = getUIUpdates();
            val composeView = findViewById<ComposeView>(R.id.composeView)
            composeView.setContent { Menu(uiState.activatedMenuItems) }

            val bkg = findViewById<LinearLayout>(R.id.menuBackground)
            if (uiState.menuVisible)
            {
                bkg.visibility = View.VISIBLE;
            } else
            {
                bkg.visibility = View.INVISIBLE
            }

            if (uiState.mapVisible)
            {
                mapView?.visibility = View.VISIBLE;
            } else
            {
                mapView?.visibility = View.INVISIBLE
            }

            // TODO: Call Plaid integration from button in Capital node command view
            if (uiState.plaidTrigger)
            {
                Log.d("GraphTask", "Init Plaid!")
                runBlocking {
                    val response = Fuel.get("https://wesxdz--start-py-create-link-token-dev.modal.run").body
                    Log.d("Plaid", "Link token responise is ${response}")
                    val linkTokenConfiguration = LinkTokenConfiguration.Builder()
                        .token(response.replace("\"", ""))
                        .build()
                    linkAccountToPlaid.launch(linkTokenConfiguration)
                }
            }

            graphView.nodes.clear()
            graphView.nodes = createArrayOfNodes()
            graphView.edges.clear()
            graphView.edges = createArrayOfEdges()
            graphView.invalidate()
            if (graphView.joystickActive)
            {
                val joyDiffX = graphView.currentJoystickX - graphView.startJoystickX;
                val joyDiffY = graphView.currentJoystickY - graphView.startJoystickY;
                setJoyDiff(joyDiffX, joyDiffY);
            } else
            {
                setJoyDiff(0.0f, 0.0f);
            }

            if (isRunning) {
                handler.postDelayed(this, 16) // Adjust delay for desired frame rate
            }
        }
    }

    private fun startGameLoop() {
        isRunning = true
        handler.post(gameLoop)
    }

    private fun stopGameLoop() {
        isRunning = false
        handler.removeCallbacks(gameLoop)
    }

    private val linkAccountToPlaid =
        registerForActivityResult(OpenPlaidLink()) {
            Log.d("Plaid",  "Register for activity")
            var accessToken = ""
            when (it) {
                is LinkSuccess -> {
                    Log.d("Plaid",  "Link success")
                    val success = it as LinkSuccess
                    val publicToken = success.publicToken
                    val webEndpoint = "https://wesxdz--start-py-token-exchange-dev.modal.run?public_token=${publicToken}"
                    // Now we need to get the access_token
                    // Use a Modal server function here...
                    runBlocking {
                        val response = Fuel.get(webEndpoint).body
                        accessToken = response
                        Log.d("Plaid", response)
                        // TODO: Save access token securely...
                        // TODO: Where to save access token/financial data?
                    }

                    // TODO: Now get the transactions..

                    val metadata = success.metadata
                    metadata.accounts.forEach { account ->
                        val accountId = account.id
                        val accountName = account.name
                        val accountMask = account.mask
                        val accountSubType = account.subtype
                    }
                    val institutionId = metadata.institution?.id
                    val institutionName = metadata.institution?.name
                    Log.d("Plaid", "${institutionName} linked!")

                    val transactionEndpoint = "https://wesxdz--start-py-get-transactions-dev.modal.run?access_token=${accessToken}"
                    runBlocking {
                        val response = Fuel.get(transactionEndpoint).body
                        Log.d("Plaid", response)
                    }

                }/* handle LinkSuccess */
                is LinkExit -> {
                    Log.d("Plaid",  "Link exit ${it}")
                }/* handle LinkExit */
            }
        }

    val default = FontFamily.SansSerif
    @OptIn(ExperimentalTextApi::class)
    val charisSIL = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
        FontFamily(
            Font(
                R.font.charis_sil,
                variationSettings = FontVariation.Settings(
                    FontVariation.weight(950),
                    FontVariation.width(30f),
                    FontVariation.slant(-6f),
                )
            )
        )
    } else {
        default
    }

    @Composable
    fun ExampleBox(shape: Shape){
        Column(modifier = Modifier
            .fillMaxWidth()
            .wrapContentSize(Alignment.Center)) {
            Box(
                modifier = Modifier
                    .size(100.dp)
                    .clip(shape)
                    .background(color = colorResource(R.color.space_cadet_grey))
            )
        }
    }


    @Composable
    fun NodeFeatureCommand(name: String, active: Boolean) {
        Column (
            modifier = Modifier.onGloballyPositioned { coordinates ->
                val height = coordinates.size.height.toFloat()
                Log.d("GraphLayout", "Graph height is ${height}")
            }
        ) {
        Row(
            modifier = Modifier
                .padding(2.dp)
                .absolutePadding(left = 32.dp, right = 0.dp)
        )
        {
            if (active) {
                Row(
                    modifier = Modifier
                        .padding(2.dp)
                        .absolutePadding(left = 12.dp, right = 8.dp)
                ) {
                    Text(
                        text = name,
                        fontFamily = charisSIL,
                        color = colorResource(R.color.black),
                        fontSize = 24.sp
                    )
                }
            } else {
                Row(
                    modifier = Modifier
                        .background(color = colorResource(R.color.dead_dream), shape = RoundedCornerShape(4.dp))
                ) {
                    Row(
                        modifier = Modifier
                            .padding(2.dp)
                            .absolutePadding(left = 12.dp, right = 8.dp)
                            .grayScale()
                    ) {
                        Text(
                            text = name,
                            fontFamily = charisSIL,
                            color = colorResource(R.color.space_cadet_grey),
                            fontSize = 24.sp
                        )
                    }
            }
        }
        }
        }
    }


    // https://stackoverflow.com/questions/75689724/set-the-entire-component-to-grayscale-compose
    class GrayScaleModifier : DrawModifier {
        override fun ContentDrawScope.draw() {
            val saturationMatrix = ColorMatrix().apply { setToSaturation(0f) }

            val saturationFilter = ColorFilter.colorMatrix(saturationMatrix)
            val paint = androidx.compose.ui.graphics.Paint().apply {
                colorFilter = saturationFilter
            }
            drawIntoCanvas {
                it.saveLayer(Rect(0f, 0f, size.width, size.height), paint)
                drawContent()
                it.restore()
            }
        }
    }
    fun Modifier.grayScale() = this.then(GrayScaleModifier())

//    @Composable
//    fun NodeOutline(){
//        ExampleBox(shape = CircleShape)
//    }
//
//    @Composable
//    fun ExampleBox(shape: Shape){
//        Box(
//            modifier = Modifier
//                .size(16.dp)
//                .clip(shape)
//                .background(colorResource(R.color.space_cadet_grey))
//                .wrapContentHeight(Alignment.CenterVertically)
//        )
//    }
//
//
//    @Composable
//    fun NodeSelector()
//    {
//        NodeOutline()
//    }


    @Composable
    fun Menu(activated: HashSet<String>) {
        Column(modifier = Modifier
            .wrapContentSize(Alignment.Center)) {
            // TODO: We need a better way to store and load GNN generated categories between flecs/Kotlin/PyG
            val categories = getString(R.string.greeting).split("\n")
            var i = 0
//            Log.d("GraphDebug", "Activated categories count is ${activated.size}")
            for (category in categories) {
                // Active map
                NodeFeatureCommand(name = category, active = activated.contains(category))
                i++
            }
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        locationMapUpdateReceiver.parentActivity = this
        setContentView(R.layout.activity_main)

        val assetManager: AssetManager = assets
        val gn = initWorld(assetManager)
        Log.d("flecs_query", "$gn");


        // This is done in xml theme now...
//        val windowInsetsController = WindowCompat.getInsetsController(window, window.decorView)
//        windowInsetsController.hide(WindowInsetsCompat.Type.systemBars())
//        getSupportActionBar()?.hide()
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        startGameLoop()

        // Before you perform the actual permission request, check whether your app
        // already has the permissions, and whether your app needs to show a permission
        // rationale dialog. For more details, see Request permissions.
        locationPermissionRequest.launch(arrayOf(
            Manifest.permission.ACCESS_FINE_LOCATION,
            Manifest.permission.ACCESS_COARSE_LOCATION))

        // Consider foreground task that visualizes a graph relating to realtime data integration
        // TODO: How to wear a 360 camera without looking dumb?
        // Start with a simple Raspberry Pi single camera...

        // TODO: Is the worker already running?
//        val workManager: WorkManager = WorkManager.getInstance(applicationContext)
//        workManager.enqueue(PeriodicWorkRequestBuilder<TrackLocationWorker>(15, TimeUnit.MINUTES).build())

        Log.d("GraphLifeServices", "Request start service")
        val intent = Intent(applicationContext, TrackLocationService::class.java)
        // TODO: Use PendingIntent, return location update, then update the mapbox location here...

        startForegroundService(intent)

        mapView = findViewById<MapView>(R.id.mapView)
        mapView?.getMapboxMap()?.loadStyleUri(Style.LIGHT)

        val graphView = findViewById<GraphView>(R.id.graphView)
        graphView.parentActivity = this;

        val bkg = findViewById<LinearLayout>(R.id.menuBackground)
        bkg.visibility = View.INVISIBLE

        val composeView = findViewById<ComposeView>(R.id.composeView)
//        Log.d("GraphDebug", "Prior to compose view set menu content: Activated categories count is ${uiState.activatedMenuItems.size}")
        composeView.setContent { Menu(uiState.activatedMenuItems) }

        // Example of a call to a native method
//        binding.sampleText.text = stringFromJNI()
    }

    /**
     * A native method that is implemented by the 'graphlife' native library,
     * which is packaged with this application.
     */
    external fun stringFromJNI(): String
    external fun setJoyDiff(x : Float, y : Float)
    external fun getPlayerPos() : GraphNode
    external fun getCameraPos() : GraphNode
    external fun initWorld(assetManager: AssetManager): Int
    // https://developer.android.com/training/articles/perf-jni
    // Consider Array Get/Release paradigm for better perf
    external fun createArrayOfNodes(): ArrayList<GraphNode>
    external fun createArrayOfEdges(): ArrayList<Edge>
    external fun stepWorld(): Int
    external fun getUIUpdates() : UIState
    external fun setUIMenuOffset(x : Float, y : Float)

    companion object {
        // Used to load the 'graphlife' library on application startup.
        init {
            System.loadLibrary("graphlife")
//            System.loadLibrary("android-game");
        }
    }
}