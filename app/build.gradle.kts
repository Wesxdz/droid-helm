plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("io.realm.kotlin") version "1.11.0"
}

val MAPBOX_DOWNLOADS_TOKEN: String by project

android {
    namespace = "com.example.graphlife"
    compileSdk = 34

//    packaging {
//        resources.excludes.add("META-INF/native-image/org.mongodb/bson/native-image.properties")
//    }
    android { packaging { resources.excludes.add("META-INF/*") } }

    defaultConfig {
        applicationId = "com.example.graphlife"
        minSdk = 26
        targetSdk = 33
        versionCode = 1
        versionName = "1.0"

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
    }

    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
        }
    }
    compileOptions {
//        sourceCompatibility = JavaVersion.VERSION_17
//        targetCompatibility = JavaVersion.VERSION_17
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    kotlinOptions {
//        jvmTarget = "17"
        jvmTarget = "1.8"
    }
    externalNativeBuild {
        cmake {
            path = file("src/main/cpp/CMakeLists.txt")
            version = "3.22.1"
        }
    }
    buildFeatures {
        viewBinding = true
        compose = true
//        prefab = true
    }

    composeOptions {
        kotlinCompilerExtensionVersion = "1.5.0"
    }
}

dependencies {
    //implementation ("androidx.games:games-frame-pacing:2.0.0")
//    implementation ("androidx.games:games-activity:2.0.2")
    //implementation ("androidx.games:games-controller:2.0.0")
    implementation("androidx.core:core-ktx:1.9.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.9.0")
    implementation("androidx.constraintlayout:constraintlayout:2.1.4")
    implementation("androidx.games:games-activity:2.0.2")
    implementation(files("app/libs/agdk-libraries-2023.3.0.0"))
    implementation("androidx.work:work-runtime-ktx:2.8.1")
    implementation("androidx.compose.ui:ui-android:1.5.1")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")
    implementation("com.google.android.gms:play-services-location:21.0.1")

    implementation("io.realm.kotlin:library-base:1.11.0")
    implementation("io.realm.kotlin:library-sync:1.11.0") // If using Device Sync
    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.7.0") // If using coroutines with the SDK

    implementation("com.plaid.link:sdk-core:3.13.2")
    implementation("com.mapbox.maps:android:10.16.0")

    // fuel
    implementation("com.github.kittinunf.fuel:fuel:3.0.0-alpha1")

    // Jetpack compose
    val composeBom = platform("androidx.compose:compose-bom:2023.08.00")
    implementation(composeBom)
    androidTestImplementation(composeBom)

    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.material:material")
    implementation("androidx.compose.foundation:foundation")

    implementation("androidx.compose.ui:ui-tooling-preview")
    debugImplementation("androidx.compose.ui:ui-tooling")

    implementation("androidx.fragment:fragment-ktx:1.6.1")
}