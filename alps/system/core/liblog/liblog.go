package mtkLogEnhance
import (
	"os"
	"path/filepath"
	"android/soong/android"
	"android/soong/cc"
	"android/soong/android/mediatek"
)

func mtkLogEnhanceDefaults(ctx android.LoadHookContext) {
	type props struct {
		Target struct {
			Android struct {
				Cflags []string
			}
		}
	}
	p := &props{}
	mtkLogEnable := filepath.Join(".", "vendor/mediatek/internal/mtklog_enable")
	if _, err := os.Stat(mtkLogEnable); err == nil {
		if mediatek.GetFeature("MTK_ANDROID_LOG_MUCH_COUNT") != "" {
			p.Target.Android.Cflags = append(p.Target.Android.Cflags, "-DANDROID_LOG_MUCH_COUNT")
		}
	}

	if (mediatek.GetFeature("MTK_LOGD_ENHANCE_DISABLE") == "") ||
		(mediatek.GetFeature("MTK_LOGD_ENHANCE_DISABLE") != "yes") {
		p.Target.Android.Cflags = append(p.Target.Android.Cflags, "-DMTK_LOGD_ENHANCE")
	}
	ctx.AppendProperties(p)
}
func init() {
	android.RegisterModuleType("mtk_log_enhance_defaults", mtkLogEnhanceDefaultsFactory)
}

func mtkLogEnhanceDefaultsFactory() android.Module {
	module := cc.DefaultsFactory()
	android.AddLoadHook(module, mtkLogEnhanceDefaults)
	return module
}

