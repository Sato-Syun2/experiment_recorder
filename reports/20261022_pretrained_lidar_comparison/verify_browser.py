"""Exercise the generated offline comparison with a temporary Chrome profile."""
from pathlib import Path
import argparse
import json
from playwright.sync_api import sync_playwright

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--comparison-dir',type=Path,default=Path(__file__).resolve().parent/'comparison')
p=parser.parse_args().comparison_dir.resolve()
errors=[]
with sync_playwright() as runner:
    browser=runner.chromium.launch(executable_path='/usr/bin/google-chrome',headless=True,args=['--no-sandbox'])
    page=browser.new_page(viewport={'width':1280,'height':1050})
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto((p/'index.html').as_uri()+'?i=20')
    page.wait_for_function('document.getElementById("image").complete && document.getElementById("image").naturalWidth > 0')
    before=page.locator('#time').inner_text()
    page.screenshot(path=str(p/'browser_10s.png'),full_page=True)
    page.locator('#next').click()
    after=page.locator('#time').inner_text()
    page.locator('#slider').evaluate('(e)=>{e.value=40;e.dispatchEvent(new Event("input"))}')
    page.wait_for_timeout(200)
    at20=page.locator('#time').inner_text()
    page.screenshot(path=str(p/'browser_20s.png'),full_page=True)
    page.locator('#play').click()
    page.wait_for_timeout(1200)
    played=page.locator('#time').inner_text()
    page.locator('#play').click()
    result=dict(js_errors=errors,initial_time=before,after_next=after,slider_time=at20,
                after_play=played,images_loaded=page.locator('#image').evaluate('(e)=>e.naturalWidth'),
                next_changes_time=before!=after,play_changes_time=at20!=played)
    browser.close()
(p/'browser_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))
if errors or not result['next_changes_time'] or not result['play_changes_time']:
    raise SystemExit(1)
